// Experimental self-level control; no position or altitude hold.
// FL7 CCW, FR5 CW, RR3 CCW, RL2 CW. MPU SDA10/SCL1, +Y FRONT, +X RIGHT.
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <esp_system.h>
#include <errno.h>
#include <ctype.h>
#include "FlightMath.h"
#include "BenchImu.h"
#include "ControlSession.h"
#include "FlightRecorder.h"
#include "PhoneLink.h"

#if !defined(CONFIG_IDF_TARGET_ESP32C3) || ESP_ARDUINO_VERSION_MAJOR != 3
#error "Select ESP32C3 Dev Module with Arduino-ESP32 core 3.x."
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Enable USB CDC On Boot."
#endif

constexpr uint8_t PINS[4]={7,5,3,2};
constexpr uint16_t OFF=1000;
constexpr float ANGLE_GAIN=3;
// Keep the prior damping/P gains. Pitch I is a modest candidate increase
// from 0.12 to 0.24 to learn the logged persistent nose-down bias faster.
flight::RatePid rollPid{0.7f,flight::kBalanceKi,0.002f},pitchPid{0.7f,flight::kPitchBalanceKi,0.002f},yawPid{0.5f,0,0};
BenchImu imu;
PhoneLink phone;
ControlSession session;
FlightRecorder recorder;
bool attached[4]={},armed=false,fault=false,limited=false,bench=false,networkReady=false;
const char *reason="calibrating; keep frame level and still";
float base=OFF,target=OFF,virtualUs[4]={OFF,OFF,OFF,OFF};
uint16_t sent[4]={OFF,OFF,OFF,OFF};
uint32_t bootTime=0,lastOutput=0,lastStatus=0,benchStart=0;
float maxSampleGapMs=0;
char rx[48]; size_t rxCount=0; bool badLine=false;
char lastReport[PhoneLink::REPORT_SIZE]="No stopped test recorded yet.\n";
size_t reportLength=0,reportOffset=0;
char diagnostic[512]={}; size_t diagnosticLength=0,diagnosticOffset=0;

void queueStatus() {
  const int n=snprintf(diagnostic,sizeof(diagnostic),
      "ready=%u fault=%u armed=%u R=%.2f P=%.2f calibration=%u/600 reason=%s\n",
      imu.ready,fault,armed,imu.attitude.roll(),imu.attitude.pitch(),imu.count,reason);
  diagnosticLength=n>0 ? min((size_t)n,sizeof(diagnostic)-1):0; diagnosticOffset=0;
}

uint32_t duty(uint16_t us) { return ((uint32_t)us*16384UL+10000UL)/20000UL; }
void resetPids() { rollPid.reset(); pitchPid.reset(); yawPid.reset(); limited=false; }
void capture(bool final=false) {
  FlightRow row;
  row.roll=imu.attitude.roll(); row.pitch=imu.attitude.pitch();
  row.rates[0]=imu.rate.x; row.rates[1]=imu.rate.y; row.rates[2]=imu.rate.z;
  row.integral[0]=rollPid.integral; row.integral[1]=pitchPid.integral; row.integral[2]=yawPid.integral;
  row.g=flight::norm(imu.accel); row.base=lroundf(base); row.target=lroundf(target); row.limited=limited;
  for (unsigned i=0;i<4;++i) row.sent[i]=sent[i];
  recorder.capture(millis(),row,final);
}
void buildReport() {
  if (!recorder.count) return;
  size_t ri=0,ra=0,pi=0,pa=0;
  for (size_t i=1;i<recorder.count;++i) {
    if (recorder.at(i).roll<recorder.at(ri).roll) ri=i;
    if (recorder.at(i).roll>recorder.at(ra).roll) ra=i;
    if (recorder.at(i).pitch<recorder.at(pi).pitch) pi=i;
    if (recorder.at(i).pitch>recorder.at(pa).pitch) pa=i;
  }
  const auto &r=recorder.at(recorder.count-1);
  const int n=snprintf(lastReport,sizeof(lastReport),
    "STOPPED: %s. Motors requested OFF.\nSaved final %u samples at 20 Hz, t=%lu..%lu ms since arm.\n"
    "Roll %.2f..%.2f deg; pitch %.2f..%.2f deg.\n"
    "FINAL BEFORE STOP: R=%.2f P=%.2f rates=%.2f %.2f %.2f g=%.2f\n"
    "Base=%u target=%u; FL7 FR5 RR3 RL2=%u %u %u %u; I=%.2f %.2f %.2f; limit=%u\n"
    "Largest fresh-sample gap=%.2f ms. Pulse requests, not measured thrust.\n",
    reason,(unsigned)recorder.count,(unsigned long)recorder.at(0).ms,(unsigned long)r.ms,
    recorder.at(ri).roll,recorder.at(ra).roll,recorder.at(pi).pitch,recorder.at(pa).pitch,
    r.roll,r.pitch,r.rates[0],r.rates[1],r.rates[2],r.g,r.base,r.target,
    r.sent[0],r.sent[1],r.sent[2],r.sent[3],r.integral[0],r.integral[1],r.integral[2],r.limited,maxSampleGapMs);
  reportLength=n>0 ? min((size_t)n,sizeof(lastReport)-1):0; reportOffset=0;
  phone.publishReport(lastReport);
}
void forceLow(unsigned i) {
  if (attached[i]) ledcDetach(PINS[i]);
  attached[i]=false; digitalWrite(PINS[i],LOW); pinMode(PINS[i],OUTPUT); digitalWrite(PINS[i],LOW);
}
void stopAll(const char *why) {
  const bool hadRun=armed;
  if (hadRun) capture(true);
  armed=bench=false; session.stop(); base=target=OFF; resetPids();
  if (!fault) reason=why;
  for (unsigned i=0;i<4;++i) {
    virtualUs[i]=OFF;
    if (attached[i] && !ledcWrite(PINS[i],duty(OFF))) {
      forceLow(i); fault=true; reason="OFF write failed; pin detached";
    }
    sent[i]=OFF;
  }
  if (hadRun) buildReport(); // All output-stop writes happen before formatting.
}
void trip(const char *why) {
  // Preserve any previously latched fault.
  if (fault) return;
  stopAll(why); fault=true;
}
bool canArm() {
  if (fault || imu.fault || !imu.ready || (uint32_t)(millis()-bootTime)<5000) return false;
  if (fabsf(imu.attitude.roll())>5 || fabsf(imu.attitude.pitch())>5 || flight::norm(imu.rate)>3) return false;
  for (bool a:attached) if (!a) return false;
  return true;
}
void beginRun() {
  armed=true; base=target=OFF; resetPids(); maxSampleGapMs=0;
  for (float &v:virtualUs) v=OFF;
  recorder.begin(millis()); reportLength=reportOffset=0;
  reason="armed; zero throttle";
}
void acceptPhone() {
  ControlPacket p;
  if (!phone.take(p)) return;
  if (p.action==0) { stopAll("phone STOP"); return; }
  if (bench || fault || imu.fault) return;
  const auto result=session.accept(p,millis(),canArm() && !armed);
  if (result==ControlSession::BLOCKED) { reason="ARM blocked: need healthy, calibrated, level and still frame at zero throttle"; return; }
  if (result==ControlSession::ARM) beginRun();
  if (result==ControlSession::UPDATE && armed) {
    target=session.throttle;
    reason=target>=1100 ? "self-level control active":"armed; motors OFF at zero throttle";
  }
}
void publishStatus() {
  char json[PhoneLink::STATUS_SIZE];
  snprintf(json,sizeof(json),
    "{\"ready\":%s,\"fault\":%s,\"armed\":%s,\"canArm\":%s,\"calibration\":%u,\"nonce\":%lu,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"base\":%u,\"target\":%u,\"motors\":[%u,%u,%u,%u],\"reason\":\"%s\"}",
    imu.ready?"true":"false",fault?"true":"false",armed?"true":"false",canArm()?"true":"false",imu.count,
    (unsigned long)session.nonce,imu.attitude.roll(),imu.attitude.pitch(),(unsigned)lroundf(base),(unsigned)lroundf(target),
    sent[0],sent[1],sent[2],sent[3],reason);
  phone.publish(json);
}
void serialCommand(char *line) {
  while (isspace((unsigned char)*line)) ++line;
  char *end=line+strlen(line); while (end>line && isspace((unsigned char)end[-1])) *--end=0;
  if (!strcmp(line,"stop") || !strcmp(line,"1000")) { stopAll("USB STOP"); return; }
  if (!strcmp(line,"status")) {
    // Do not format serial diagnostics while a motor run is active.
    if (!armed) queueStatus();
    return;
  }
  if (!strcmp(line,"report")) { if (!armed) reportOffset=0; return; }
  if (!strcmp(line,"cal")) {
    stopAll("recalibrating");
    if (!fault) imu.recalibrate();
    return;
  }
  int pulse=0,n=-1;
  if (sscanf(line,"test %d%n",&pulse,&n)==1 && n==(int)strlen(line) && pulse>=1100 && pulse<=1700) {
    if (armed || !canArm()) {
      if (!armed) { if (!fault) reason="Test blocked: wait for healthy level calibration"; queueStatus(); }
      return;
    }
    // Explicit single-command, bounded bare-motor bench run. Never auto-starts.
    session.stop(); beginRun(); bench=true; benchStart=millis(); target=pulse;
    reason="10 s USB bench test; props OFF"; return;
  }
  if (!*line) return;
  stopAll("unknown USB command");
}
void readSerial() {
  unsigned budget=32;
  while (budget-- && Serial.available()) {
    const int c=Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      if (badLine) stopAll("invalid USB line");
      else { rx[rxCount]=0; serialCommand(rx); }
      rxCount=0; badLine=false;
    } else if (c<32 || c>126 || rxCount>=sizeof(rx)-1) badLine=true;
    else if (!badLine) rx[rxCount++]=(char)c;
  }
}
void enforceLink() {
  if (!armed) return;
  if (bench && (uint32_t)(millis()-benchStart)>=10000) stopAll("10 s bench test complete");
  else if (!bench && session.expired(millis())) stopAll("phone command timeout >500 ms");
}
void stabilize() {
  if (!armed) return;
  const float roll=imu.attitude.roll(),pitch=imu.attitude.pitch();
  maxSampleGapMs=fmaxf(maxSampleGapMs,imu.dt*1000);
  if (fabsf(roll)>35 || fabsf(pitch)>35) { stopAll("tilt exceeded 35 degrees"); return; }
  if (target<1100) {
    base=OFF; resetPids(); for (float &v:virtualUs) v=OFF;
    return;
  }
  const float slew=(target>=base ? 200.0f:600.0f)*imu.dt;
  base+=flight::clamp(target-base,-slew,slew);
  if (base<1100) { resetPids(); for (float &v:virtualUs) v=OFF; return; }
  const auto m=flight::levelMix(base,roll,pitch,imu.rate,imu.dt,ANGLE_GAIN,
      rollPid,pitchPid,yawPid,!limited,bench?0:session.roll,bench?0:session.pitch,bench?0:session.yaw);
  limited=m.limited;
  for (unsigned i=0;i<4;++i) virtualUs[i]=m.pulse[i];
}
void writeOutputs() {
  if ((uint32_t)(micros()-lastOutput)<20000) return;
  lastOutput=micros();
  for (unsigned i=0;i<4;++i) {
    const uint16_t us=armed && !fault ? (uint16_t)lroundf(virtualUs[i]):OFF;
    if (!attached[i] || !ledcWrite(PINS[i],duty(us))) { trip("LEDC write failed"); return; }
    sent[i]=us;
  }
}
void setup() {
  for (unsigned i=0;i<4;++i) { digitalWrite(PINS[i],LOW); pinMode(PINS[i],OUTPUT); }
  Serial.begin(115200);
  session.nonce=esp_random()|1u;
  if (!flight::selfTest()) { trip("controller math self-test failed"); return; }
  for (unsigned i=0;i<4;++i) {
    attached[i]=ledcAttachChannel(PINS[i],50,14,i);
    if (!attached[i] || !ledcWrite(PINS[i],duty(OFF))) { forceLow(i); trip("PWM initialization failed"); return; }
  }
  networkReady=phone.begin();
  if (!networkReady) { trip("phone network initialization failed"); return; }
  // HTTP formatting/receives run at lower priority than this control task.
  vTaskPrioritySet(nullptr,5);
  bootTime=millis();
  Serial.printf("StabilizeNow experimental. DISARMED. Reset=%d.\n",(int)esp_reset_reason());
  Serial.println("Keep frame level and still. Phone Wi-Fi QuadLevel / CHANGE_ME_123; open http://192.168.4.1");
  Serial.println("USB props-off test: test 1400 (10 s, auto report). stop | status | report | cal");
  if (!imu.begin()) { trip(imu.fault); queueStatus(); }
}
void loop() {
  enforceLink(); readSerial(); acceptPhone(); enforceLink();
  if (!fault) {
    const bool wasReady=imu.ready;
    const bool fresh=imu.tick();
    if (imu.fault) trip(imu.fault);
    else if (!wasReady && imu.ready) { reason="READY; calibrated level reference; motors OFF"; queueStatus(); }
    if (fresh && !fault) stabilize();
    enforceLink();
    if (!fault) writeOutputs();
    if (fresh && armed && !fault) capture();
  }
  if ((uint32_t)(millis()-lastStatus)>=100) { lastStatus=millis(); publishStatus(); }
  if (!armed && reportOffset<reportLength) {
    const int room=Serial.availableForWrite();
    if (room>0) {
      const size_t n=min((size_t)room,min((size_t)64,reportLength-reportOffset));
      reportOffset+=Serial.write((const uint8_t *)lastReport+reportOffset,n);
    }
  } else if (!armed && diagnosticOffset<diagnosticLength) {
    const int room=Serial.availableForWrite();
    if (room>0) {
      const size_t n=min((size_t)room,min((size_t)64,diagnosticLength-diagnosticOffset));
      diagnosticOffset+=Serial.write((const uint8_t *)diagnostic+diagnosticOffset,n);
    }
  }
  delay(1);
}
