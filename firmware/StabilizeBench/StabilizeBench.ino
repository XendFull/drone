// SELF-LEVEL BENCH PROTOTYPE, NOT FLIGHT-READY. REMOVE ALL PROPELLERS.
// Top/front: FL=GPIO7 CCW, FR=GPIO5 CW, RR=GPIO3 CCW, RL=GPIO2 CW.
// MPU: 3V3, common GND, SDA10, SCL1, sensor +Y points toward drone front.
// Startup PREVIEW: real outputs remain at 1000us. 'bench' enables bare-motor tests.
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <esp_system.h>
#include <errno.h>
#include <stdarg.h>
#include <ctype.h>
#include "FlightMath.h"
#include "BenchImu.h"
#include "BenchRecorder.h"

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Select ESP32C3 Dev Module."
#endif
#if ESP_ARDUINO_VERSION_MAJOR != 3
#error "Use Espressif Arduino-ESP32 Core 3.x (project uses 3.3.10)."
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot > Enabled."
#endif

constexpr uint8_t PINS[4]={7,5,3,2}; // FL, FR, RR, RL, not old m1..m4 order.
constexpr uint16_t OFF=1000;
constexpr uint32_t START_HOLD_MS=5000, LEASE_MS=5000, MAX_TEST_MS=15000;
constexpr float ANGLE_GAIN=3.0f; // degrees/s requested per degree of tilt.
// Candidate bench tuning. Small bounded I counters persistent roll/pitch bias.
flight::RatePid rollPid{0.7f,flight::kBalanceKi,0.002f};
flight::RatePid pitchPid{0.7f,flight::kBalanceKi,0.002f};
flight::RatePid yawPid{0.5f,0.0f,0.0f};
bool balanceEnabled=true;
BenchImu imu;
bool attached[4]={}, armed=false, preview=true, fault=false, limited=false;
bool streamLogs=false;
const char *reason="startup";
esp_reset_reason_t bootResetReason=ESP_RST_UNKNOWN;
float requestedBase=OFF, actualBase=OFF;
float virtualUs[4]={OFF,OFF,OFF,OFF};
uint16_t writtenUs[4]={OFF,OFF,OFF,OFF};
uint32_t bootTime=0, lastCommand=0, armTime=0, lastOutput=0, lastLog=0;
char rx[40]; size_t rxLength=0; bool badLine=false;
char tx[2048]; size_t txHead=0,txTail=0; uint32_t droppedLogs=0;
BenchRecorder recorder;
const char *recordStopReason="none";
bool recordedPreview=true;
enum ReportMode { REPORT_NONE, REPORT_SUMMARY, REPORT_FULL };
ReportMode reportMode=REPORT_NONE;
size_t reportStep=0, extremeRows[4]={};

void captureRecord(bool final) {
  BenchRecord row={};
  row.roll=imu.attitude.roll(); row.pitch=imu.attitude.pitch();
  row.rates[0]=imu.rate.x; row.rates[1]=imu.rate.y; row.rates[2]=imu.rate.z;
  row.accelG=flight::norm(imu.accel);
  row.base=(uint16_t)lroundf(actualBase); row.target=(uint16_t)lroundf(requestedBase);
  for (unsigned i=0;i<4;++i) {
    row.calculated[i]=(uint16_t)lroundf(virtualUs[i]); row.sent[i]=writtenUs[i];
  }
  row.limited=limited;
  row.integral[0]=rollPid.integral; row.integral[1]=pitchPid.integral; row.integral[2]=yawPid.integral;
  recorder.capture(millis(),row,final);
}

void say(const char *format,...) {
  char line[300]; va_list args;
  va_start(args,format); vsnprintf(line,sizeof(line),format,args); va_end(args);
  const size_t n=strlen(line), used=(txHead+sizeof(tx)-txTail)%sizeof(tx);
  if (n>sizeof(tx)-1-used) { ++droppedLogs; return; }
  for (size_t i=0;i<n;++i) { tx[txHead]=line[i]; txHead=(txHead+1)%sizeof(tx); }
}
const char *resetName() {
  switch (bootResetReason) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "external reset";
    case ESP_RST_SW: return "software restart";
    case ESP_RST_PANIC: return "crash/panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep wake";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "SDIO reset";
    case ESP_RST_USB: return "USB reset";
    case ESP_RST_JTAG: return "JTAG reset";
    case ESP_RST_EFUSE: return "eFuse error";
    case ESP_RST_PWR_GLITCH: return "power glitch";
    case ESP_RST_CPU_LOCKUP: return "CPU lockup";
    default: return "unknown";
  }
}
void bootStatus() {
  say("Reset reason: %s (%d); uptime=%lu ms.\n",resetName(),(int)bootResetReason,(unsigned long)millis());
}
void drainLog() {
  const int available=Serial.availableForWrite();
  size_t budget=available>0 ? min((size_t)available,(size_t)64) : 0;
  while (budget-- && txTail!=txHead) {
    if (Serial.write((uint8_t)tx[txTail])!=1) break;
    txTail=(txTail+1)%sizeof(tx);
  }
}
uint32_t duty(uint16_t us) { return ((uint32_t)us*16384UL+10000UL)/20000UL; }
void printRecord(size_t index,const char *label) {
  const BenchRecord &r=recorder.rows[index];
  say("%s t=%lu ms R=%.1f P=%.1f rates=%.1f %.1f %.1f g=%.2f base=%u/%u calc=%u,%u,%u,%u sent=%u,%u,%u,%u I=%.1f,%.1f,%.1f limit=%u\n",
      label,(unsigned long)r.ms,r.roll,r.pitch,r.rates[0],r.rates[1],r.rates[2],r.accelG,r.base,r.target,
      r.calculated[0],r.calculated[1],r.calculated[2],r.calculated[3],r.sent[0],r.sent[1],r.sent[2],r.sent[3],
      r.integral[0],r.integral[1],r.integral[2],r.limited);
}
void pumpReport() {
  // Stop first; queue at most one short line per loop. Slow/disconnected USB
  // cannot block control, and a full queue defers rather than drops report rows.
  if (armed || reportMode==REPORT_NONE || !recorder.count) return;
  const size_t used=(txHead+sizeof(tx)-txTail)%sizeof(tx);
  if (sizeof(tx)-1-used<300) return;
  if (reportMode==REPORT_FULL) {
    if (!reportStep) say("SAVED TRACE: %s; software pulse requests, order FL7 FR5 RR3 RL2.\n",recordStopReason);
    else if (reportStep<=recorder.count) printRecord(reportStep-1,"TRACE");
    else { say("END TRACE. Motors OFF. report reprints the short summary.\n"); reportMode=REPORT_NONE; }
    ++reportStep; return;
  }
  if (reportStep==0) {
    for (unsigned j=0;j<4;++j) extremeRows[j]=0;
    for (size_t i=1;i<recorder.count;++i) {
      const BenchRecord &r=recorder.rows[i];
      if (r.roll<recorder.rows[extremeRows[0]].roll) extremeRows[0]=i;
      if (r.roll>recorder.rows[extremeRows[1]].roll) extremeRows[1]=i;
      if (r.pitch<recorder.rows[extremeRows[2]].pitch) extremeRows[2]=i;
      if (r.pitch>recorder.rows[extremeRows[3]].pitch) extremeRows[3]=i;
    }
    say("TEST REPORT: motors OFF; stopped: %s; %s; %u samples over %lu ms.\n",recordStopReason,
        recordedPreview?"PREVIEW (outputs OFF)":"BENCH",(unsigned)recorder.count,(unsigned long)recorder.rows[recorder.count-1].ms);
  } else if (reportStep==1) {
    say("Recorded roll range %.1f..%.1f deg; pitch %.1f..%.1f deg.\n",
        recorder.rows[extremeRows[0]].roll,recorder.rows[extremeRows[1]].roll,
        recorder.rows[extremeRows[2]].pitch,recorder.rows[extremeRows[3]].pitch);
  } else if (reportStep>=2 && reportStep<=5) {
    const char *labels[]={"MIN-ROLL","MAX-ROLL","MIN-PITCH","MAX-PITCH"};
    printRecord(extremeRows[reportStep-2],labels[reportStep-2]);
  } else if (reportStep==6) printRecord(recorder.count-1,"FINAL-BEFORE-STOP");
  else {
    say("END REPORT. report reprints this; trace prints the saved 20 Hz sequence. No timed typing needed.\n");
    reportMode=REPORT_NONE;
  }
  ++reportStep;
}
void lowPin(unsigned i) {
  if (attached[i]) ledcDetach(PINS[i]);
  attached[i]=false;
  digitalWrite(PINS[i],LOW); pinMode(PINS[i],OUTPUT); digitalWrite(PINS[i],LOW);
}
void resetPids() { rollPid.reset(); pitchPid.reset(); yawPid.reset(); limited=false; }
void stopAll(const char *why) {
  const bool finishedTest=armed;
  if (finishedTest) captureRecord(true); // Preserve pre-stop tilt and demands.
  armed=false; requestedBase=actualBase=OFF;
  if (!fault) reason=why; // Preserve the latched fault across mode/stop commands.
  resetPids();
  for (unsigned i=0;i<4;++i) {
    virtualUs[i]=OFF;
    if (attached[i] && !ledcWrite(PINS[i],duty(OFF))) {
      lowPin(i); fault=true; reason="OFF write failed";
    }
    writtenUs[i]=OFF;
  }
  if (finishedTest) {
    recordStopReason=reason; reportStep=0; reportMode=REPORT_SUMMARY;
  }
}
void trip(const char *why) {
  stopAll(why); fault=true; reason=why;
  say("FAULT: %s. Reset after investigating.\n",why);
}
void status() {
  bootStatus();
  say("Balance correction=%s; I(us)=%.1f %.1f %.1f; roll/pitch Ki=%.2f; bounded +/-25 us, active above 1250 base.\n",
      balanceEnabled?"ON":"OFF",rollPid.integral,pitchPid.integral,yawPid.integral,rollPid.ki);
  say("%s armed=%u ready=%u fault=%u reason=%s base=%.0f target=%.0f\n",
      preview?"PREVIEW (motors OFF)":"BENCH (motor output enabled)",armed,imu.ready,fault,reason,actualBase,requestedBase);
  say("roll=%.1f pitch=%.1f rates=%.2f %.2f %.2f calibration=%u/600 restarts=%lu face=%d\n",
      imu.attitude.roll(),imu.attitude.pitch(),imu.rate.x,imu.rate.y,imu.rate.z,
      imu.count,(unsigned long)imu.restarts,imu.face);
  if (imu.fault) say("MPU FAULT: %s\n",imu.fault);
  else if (!imu.ready) say("Calibration: %s; last restart: %s\n",imu.calibrationWait,imu.lastRestart);
  say("FL7 FR5 RR3 RL2 calculated=%.0f %.0f %.0f %.0f sent=%u %u %u %u limited=%u dropped=%lu\n",
      virtualUs[0],virtualUs[1],virtualUs[2],virtualUs[3],writtenUs[0],writtenUs[1],writtenUs[2],writtenUs[3],limited,(unsigned long)droppedLogs);
}
void checkCorrections() {
  // Stop/disarm first. This command never arms or sends the calculated pulses.
  stopAll("one-shot correction check"); preview=true; streamLogs=false;
  if (fault || imu.fault) {
    say("CHECK BLOCKED: %s\n",imu.fault?imu.fault:reason); return;
  }
  if (!imu.ready) {
    say("CHECK BLOCKED: calibration %u/600; %s\n",imu.count,imu.calibrationWait); return;
  }
  const float roll=imu.attitude.roll(), pitch=imu.attitude.pitch();
  const flight::Mix m=flight::levelMix(1400,roll,pitch,imu.rate,imu.dt,ANGLE_GAIN,
                                     rollPid,pitchPid,yawPid,false);
  resetPids();
  const float lr=(m.pulse[1]+m.pulse[2]-m.pulse[0]-m.pulse[3])/2;
  const float fb=(m.pulse[0]+m.pulse[1]-m.pulse[2]-m.pulse[3])/2;
  say("CHECK ONLY: motors OFF, disarmed. roll=%.1f pitch=%.1f\n",roll,pitch);
  if (fabsf(lr)<2) say("ROLL: left/right nearly equal.\n");
  else say("ROLL: %s pair would be HIGHER by %.0f us.\n",lr>0?"RIGHT (GPIO5/3)":"LEFT (GPIO7/2)",fabsf(lr));
  if (fabsf(fb)<2) say("PITCH: front/rear nearly equal.\n");
  else say("PITCH: %s pair would be HIGHER by %.0f us.\n",fb>0?"FRONT (GPIO7/5)":"REAR (GPIO3/2)",fabsf(fb));
  say("Calculated only: FL7=%.0f FR5=%.0f RR3=%.0f RL2=%.0f. Actual outputs stay 1000.\n",
      m.pulse[0],m.pulse[1],m.pulse[2],m.pulse[3]);
}
void help() {
  say("Props OFF. Frame level/still; sensor +Y FRONT. Monitor 115200, New Line.\n");
  say("check | preview | bench | arm | 1400 | 1700 | stop | keep | status | report | trace | balance on/off | cal | logs on/off | help\n");
  say("check: one-shot calculated corrections with motors OFF; no arm/throttle needed.\n");
  say("PREVIEW is default. bench stops/disarms then enables physical bare-motor testing.\n");
  say("5 s command lease; keep renews. Max run 15 s. status does NOT renew.\n");
  say("Automatic 20 Hz recording while armed; report AFTER stop. report/trace replay it while disarmed.\n");
}
void command(char *line) {
  while (isspace((unsigned char)*line)) ++line;
  char *end=line+strlen(line);
  while (end>line && isspace((unsigned char)end[-1])) *--end=0;
  for (char *p=line;*p;++p) *p=tolower((unsigned char)*p);
  if (!*line) return;
  if (!strcmp(line,"stop") || !strcmp(line,"1000")) {
    stopAll("operator stop"); say("STOP: all outputs 1000, disarmed.\n"); return;
  }
  if (!strcmp(line,"status")) { status(); return; }
  if (!strcmp(line,"balance on") || !strcmp(line,"balance off")) {
    if (armed) { say("Send stop before changing balance correction.\n"); return; }
    balanceEnabled=!strcmp(line,"balance on");
    rollPid.ki=pitchPid.ki=balanceEnabled?flight::kBalanceKi:0;
    resetPids();
    say("Balance correction %s. Disarmed; integral reset. Candidate bench tuning.\n",balanceEnabled?"ON":"OFF");
    return;
  }
  if (!strcmp(line,"report") || !strcmp(line,"trace")) {
    if (armed) { say("Recording automatically. Wait for stop to read saved report.\n"); return; }
    if (!recorder.count) { say("No recorded test yet.\n"); return; }
    reportMode=!strcmp(line,"trace")?REPORT_FULL:REPORT_SUMMARY; reportStep=0; return;
  }
  if (!strcmp(line,"check")) { checkCorrections(); return; }
  if (!strcmp(line,"logs off")) {
    streamLogs=false; txHead=txTail; say("Quiet mode. status prints once; logs on starts streaming.\n"); return;
  }
  if (!strcmp(line,"logs on")) {
    streamLogs=true; lastLog=millis(); say("Streaming ON. logs off stops streaming.\n"); return;
  }
  if (!strcmp(line,"help")) { help(); return; }
  if (!strcmp(line,"preview") || !strcmp(line,"bench")) {
    stopAll("mode changed; disarmed"); preview=strcmp(line,"bench")!=0;
    say("%s selected. Send arm only with ALL PROPS REMOVED.\n",preview?"PREVIEW: real motors OFF":"BENCH: physical motor outputs enabled"); return;
  }
  if (!strcmp(line,"cal")) {
    stopAll("recalibrating"); preview=true;
    if (fault || imu.fault) { say("Fault latched: reset to retry.\n"); return; }
    imu.recalibrate(); say("PREVIEW. Frame LEVEL and STILL for 3 seconds; sensor +Y FRONT.\n"); return;
  }
  if (!strcmp(line,"arm")) {
    if (armed) { say("Already armed; keep renews the command lease.\n"); return; }
    if (fault || imu.fault) {
      say("ARM BLOCKED: %s. Motor outputs remain OFF.\n",imu.fault?imu.fault:reason); return;
    }
    if (!imu.ready) {
      say("ARM BLOCKED: calibration %u/600; %s. Last restart: %s. Keep frame still; status prints once.\n",
          imu.count,imu.calibrationWait,imu.lastRestart); return;
    }
    if ((uint32_t)(millis()-bootTime)<START_HOLD_MS) { say("Arm rejected: startup hold 5 s.\n"); return; }
    if (fabsf(imu.attitude.roll())>8 || fabsf(imu.attitude.pitch())>8) { say("Arm rejected: return frame to calibrated level.\n"); return; }
    for (bool a:attached) if (!a) { say("Arm rejected: an ESC output is detached.\n"); return; }
    stopAll("armed; awaiting throttle");
    if (fault) { say("Arm aborted: OFF write failed. Reset after checking hardware.\n"); return; }
    armed=true; armTime=lastCommand=millis();
    recorder.begin(armTime); recordedPreview=preview; reportMode=REPORT_NONE;
    say("ARMED %s. Send 1400 or 1700. Lease 5 s; absolute test limit 15 s.\n",preview?"PREVIEW (motors remain OFF)":"BENCH (PROPS OFF)"); return;
  }
  if (!strcmp(line,"keep")) {
    if (armed) { lastCommand=millis(); say("Lease renewed; absolute 15 s test limit remains.\n"); }
    else say("Disarmed. Select mode, then arm.\n");
    return;
  }
  errno=0; char *tail=nullptr; const long us=strtol(line,&tail,10);
  if (errno || tail==line || *tail || us<1100 || us>1700) {
    stopAll("invalid command"); say("Invalid command: stopped. Throttle range 1100..1700. help lists commands.\n"); return;
  }
  if (!armed) { say("Disarmed: send arm first.\n"); return; }
  requestedBase=(float)us; lastCommand=millis();
  say("Base requested %ld us; stabilization varies individual motors, cap 1800 us.\n",us);
}
void readCommands() {
  unsigned budget=32;
  while (budget-- && Serial.available()) {
    const int c=Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      if (badLine) { stopAll("invalid/oversized line"); say("Bad line: stopped.\n"); }
      else { rx[rxLength]=0; command(rx); }
      rxLength=0; badLine=false;
    } else if (c<32 || c>126 || rxLength>=sizeof(rx)-1) badLine=true;
    else if (!badLine) rx[rxLength++]=(char)c;
  }
}
void enforceLease() {
  if (!armed) return;
  if ((uint32_t)(millis()-lastCommand)>=LEASE_MS) {
    stopAll("5 s command timeout"); say("Command timeout: stopped, disarmed.\n");
  } else if ((uint32_t)(millis()-armTime)>=MAX_TEST_MS) {
    stopAll("15 s test completed"); say("Test time limit: stopped, disarmed.\n");
  }
}
void stabilize() {
  if (!armed) return;
  const float roll=imu.attitude.roll(), pitch=imu.attitude.pitch();
  if (fabsf(roll)>35 || fabsf(pitch)>35) {
    stopAll("tilt >35 degrees"); say("Excessive tilt: stopped, disarmed.\n"); return;
  }
  const float step=200*imu.dt; // Slew base at 200 us/s; not an instant throttle jump.
  actualBase+=flight::clamp(requestedBase-actualBase,-step,step);
  if (actualBase<1100) {
    resetPids();
    for (float &v:virtualUs) v=OFF;
    return;
  }
  const flight::Mix m=flight::levelMix(actualBase,roll,pitch,imu.rate,imu.dt,ANGLE_GAIN,
                                     rollPid,pitchPid,yawPid,!limited);
  limited=m.limited;
  for (unsigned i=0;i<4;++i) virtualUs[i]=m.pulse[i];
}
void writeOutputs() {
  if ((uint32_t)(micros()-lastOutput)<20000) return; // Actual ESCs remain 50 Hz.
  lastOutput=micros();
  for (unsigned i=0;i<4;++i) {
    const uint16_t us=armed && !preview && !fault ? (uint16_t)lroundf(virtualUs[i]) : OFF;
    if (!attached[i] || !ledcWrite(PINS[i],duty(us))) { trip("LEDC output write failed"); return; }
    writtenUs[i]=us;
  }
}
void setup() {
  for (unsigned i=0;i<4;++i) { digitalWrite(PINS[i],LOW); pinMode(PINS[i],OUTPUT); }
  bootResetReason=esp_reset_reason();
  Serial.begin(115200);
  const uint32_t start=millis();
  while (!Serial && (uint32_t)(millis()-start)<2500) delay(1);
  bootStatus();
  if (!flight::selfTest()) { trip("controller math self-test"); return; }
  for (unsigned i=0;i<4;++i) {
    attached[i]=ledcAttachChannel(PINS[i],50,14,i);
    if (!attached[i] || !ledcWrite(PINS[i],duty(OFF))) { lowPin(i); trip("LEDC attach/start failed"); return; }
  }
  bootTime=millis();
  say("StabilizeBench QUIET v6: BOUNDED BALANCE CORRECTION. PREVIEW; motors OFF. help lists commands.\n");
  say("Keep frame LEVEL and STILL, sensor +Y FRONT. Wait for READY. status prints once.\n");
  if (!imu.begin()) trip(imu.fault);
}
void loop() {
  drainLog(); enforceLease(); readCommands(); enforceLease();
  if (!fault) {
    const bool wasReady=imu.ready;
    const bool fresh=imu.tick();
    if (imu.fault) trip(imu.fault);
    else if (!wasReady && imu.ready) {
      say("READY. Level reference captured. Gyro bias(raw axes)=%.2f %.2f %.2f; sensor X inferred %s.\n",
          imu.bias[0],imu.bias[1],imu.bias[2],imu.face>0?"RIGHT":"LEFT");
      say("Verify R increases with RIGHT side lowered; P increases with NOSE raised. Props OFF.\n");
    }
    if (fresh && !fault) stabilize();
    enforceLease();
    if (!fault) writeOutputs();
    if (fresh && armed && !fault) captureRecord(false);
  }
  pumpReport();
  if (streamLogs && (uint32_t)(millis()-lastLog)>=500) { lastLog=millis(); status(); }
  delay(1);
}
