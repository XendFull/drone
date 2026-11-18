// Experimental self-level control; no position or altitude hold.
// C6: FL0 CCW, FR19 CW, RR18 CCW, RL3 CW. MPU SDA6/SCL7, +X FRONT, +Y LEFT.
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <esp_system.h>
#include <errno.h>
#include <ctype.h>
#include <stdarg.h>
#include <Preferences.h>
#include "FlightMath.h"
#include "BenchImu.h"
#include "ControlSession.h"
#include "FlightRecorder.h"
#include "WobbleSummary.h"
#include "PostStopTrack.h"
#include "BenchBatch.h"
#include "AutoCompare.h"
#include "ResponseProbe.h"
#include "LevelHold.h"
#include "HoldTrace.h"
#include "YawTrace.h"
#include "TuneSettings.h"
#include "PhoneLink.h"

#if !defined(CONFIG_IDF_TARGET_ESP32C6) || ESP_ARDUINO_VERSION_MAJOR != 3
#error "Select ESP32C6 Dev Module with Arduino-ESP32 core 3.x."
#endif
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Enable USB CDC On Boot."
#endif

constexpr uint8_t PINS[4]={0,19,18,3};
constexpr uint16_t OFF=1000;
constexpr uint32_t BENCH_DURATION_MS=AutoCompare::DURATION_MS;
constexpr float ANGLE_GAIN=3;
// Factory defaults. Verified automatic results replace these runtime settings;
// experiments stay temporary until the search and references pass.
flight::RatePid rollPid{flight::kRollKp,flight::kBalanceKi,0.002f},pitchPid{0.7f,flight::kPitchBalanceKi,0.002f},yawPid{0.5f,flight::kYawBalanceKi,0};
BenchImu imu;
PhoneLink phone;
ControlSession session;
FlightRecorder recorder;
PostStopTrack postStop;
BenchBatch batch;
AutoCompare tuner;
ResponseProbe probe;
LevelHold holder;
TuneSettings settings;
OnlineSettings onlineSettings;
bool useOnlineStore=false,wifiBench=false,lastRunWifi=false;
uint32_t wifiDurationMs=7000,lastRunDurationMs=7000,onlineAckSession=0,onlineAckSequence=0;
const char *onlineResult="Ready; manual tuning is unverified";
bool savePending=false,acceptedTune=false,factorySavePending=false;
const char *storageStatus="factory defaults";
UsbPresence usbGuard;
bool usbRawConnected=false,usbPresent=true;
bool attached[4]={},armed=false,fault=false,limited=false,bench=false,networkReady=false;
bool motorCheck=false,lastRunMotorCheck=false;
const char *reason="calibrating; keep frame level and still";
float base=OFF,target=OFF,virtualUs[4]={OFF,OFF,OFF,OFF};
uint16_t sent[4]={OFF,OFF,OFF,OFF};
uint32_t bootTime=0,lastOutput=0,lastStatus=0,benchStart=0;
uint32_t tuneRevision=0;
uint32_t lastControlMs=0;
bool controlValid=false,holdTracePending=false,holdTraceReady=false;
float maxSampleGapMs=0;
char rx[48]; size_t rxCount=0; bool badLine=false;
char lastReport[PhoneLink::REPORT_SIZE]="No stopped test recorded yet.\n";
size_t reportLength=0,reportOffset=0;
char diagnostic[PhoneLink::TUNE_SIZE]={}; size_t diagnosticLength=0,diagnosticOffset=0;

void restoreGains() {
  const auto r=tuner.active?tuner.gains(AutoCompare::ROLL):settings.roll;
  const auto p=tuner.active?tuner.gains(AutoCompare::PITCH):settings.pitch;
  rollPid.kp=r.p; rollPid.ki=r.i; rollPid.kd=r.d;
  pitchPid.kp=holder.active?holder.trialPitchP:p.p; pitchPid.ki=p.i; pitchPid.kd=p.d;
  rollPid.integralLimit=settings.rollCap; pitchPid.integralLimit=settings.pitchCap;
  yawPid.kp=onlineSettings.yaw.p; yawPid.ki=onlineSettings.yaw.i; yawPid.kd=onlineSettings.yaw.d;
  yawPid.integralLimit=onlineSettings.yawCap;
}
void queueOnlineSettings() {
  char json[PhoneLink::SETTINGS_SIZE];
  snprintf(json,sizeof(json),"{\"values\":[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f],\"storage\":\"%s\",\"ackSession\":%lu,\"ackSequence\":%lu,\"result\":\"%s\"}",
    settings.roll.p,settings.roll.i,settings.roll.d,settings.pitch.p,settings.pitch.i,settings.pitch.d,
    onlineSettings.yaw.p,onlineSettings.yaw.i,onlineSettings.yaw.d,settings.rollCap,settings.pitchCap,
    onlineSettings.yawCap,onlineSettings.angleGain,onlineSettings.rollTrim,onlineSettings.pitchTrim,
    storageStatus,(unsigned long)onlineAckSession,(unsigned long)onlineAckSequence,onlineResult);
  phone.publishSettings(json);
}
void queueTuneReport() {
  const int n=tuner.format(diagnostic,sizeof(diagnostic));
  diagnosticLength=n>0 ? min((size_t)n,sizeof(diagnostic)-1):0;
  const int more=snprintf(diagnostic+diagnosticLength,sizeof(diagnostic)-diagnosticLength,
      "ACTIVE R P/I/D=%.3f/%.3f/%.4f; P P/I/D=%.3f/%.3f/%.4f\n"
      "Icaps=%.0f,%.0f accepted=%u storage=%s\n",
      settings.roll.p,settings.roll.i,settings.roll.d,settings.pitch.p,settings.pitch.i,settings.pitch.d,
      settings.rollCap,settings.pitchCap,acceptedTune,storageStatus);
  if (more>0) diagnosticLength+=min((size_t)more,sizeof(diagnostic)-diagnosticLength-1);
  diagnosticOffset=0;
  phone.publishTuneReport(diagnostic);
  ++tuneRevision;
  queueOnlineSettings();
}
void loadSettings() {
  settings=TuneSettings{}; settings.seal();
  onlineSettings=OnlineSettings{};onlineSettings.seal();useOnlineStore=false;
  Preferences prefs;
  if (!prefs.begin("quad-auto",false)) return;
  const size_t onlineSize=prefs.getBytesLength("web1");
  if(onlineSize){
    OnlineSettings storedOnline;
    OnlineSettings::Legacy oldOnline;
    const bool current=onlineSize==sizeof(storedOnline)&&prefs.getBytes("web1",&storedOnline,sizeof(storedOnline))==sizeof(storedOnline)&&storedOnline.valid();
    const bool migrated=!current&&onlineSize==sizeof(oldOnline)&&prefs.getBytes("web1",&oldOnline,sizeof(oldOnline))==sizeof(oldOnline)&&storedOnline.importLegacy(oldOnline);
    if(current||migrated){
      onlineSettings=storedOnline;settings=onlineSettings.pid;useOnlineStore=true;storageStatus="loaded online settings; tuning unverified";
    } else {useOnlineStore=true;storageStatus="invalid online record; factory defaults";}
    prefs.end();return;
  }
  TuneSettings stored;
  if (prefs.getBytesLength("pid7")==sizeof(stored) &&
      prefs.getBytes("pid7",&stored,sizeof(stored))==sizeof(stored) && stored.valid()) {
    settings=stored; storageStatus="loaded saved gains";
  } else if (prefs.getBytesLength("pid7")==0) {
    TuneSettings::Legacy legacy;
    if (prefs.getBytesLength("gains")==sizeof(legacy) &&
        prefs.getBytes("gains",&legacy,sizeof(legacy))==sizeof(legacy) && settings.importLegacy(legacy))
      storageStatus="validated earlier gains imported; new search not yet saved";
  }
  prefs.end();
}
void processSettingsSave() {
  if (!savePending || ((fault || imu.fault) && !factorySavePending) || armed || batch.active || postStop.active ||
      (usbRawConnected && (reportOffset<reportLength || diagnosticOffset<diagnosticLength))) return;
  settings.seal(); bool saved=false;
  Preferences prefs;
  if (prefs.begin("quad-auto",false)) {
    if(useOnlineStore){
      onlineSettings.pid=settings;onlineSettings.seal();OnlineSettings check;
      saved=onlineSettings.valid() && prefs.putBytes("web1",&onlineSettings,sizeof(onlineSettings))==sizeof(onlineSettings) &&
        prefs.getBytes("web1",&check,sizeof(check))==sizeof(check)&&check.valid()&&memcmp(&check,&onlineSettings,sizeof(check))==0;
    } else {
    TuneSettings check;
    saved=prefs.putBytes("pid7",&settings,sizeof(settings))==sizeof(settings) &&
        prefs.getBytes("pid7",&check,sizeof(check))==sizeof(check) &&
        check.valid() && memcmp(&check,&settings,sizeof(check))==0;
    }
    prefs.end();
  }
  // No sensor-gap exception is made while armed. Flash writes happen OFF only.
  imu.resyncDisarmedClock();
  const bool wasFactoryReset=factorySavePending;
  savePending=factorySavePending=false;
  storageStatus=saved?(wasFactoryReset?"FACTORY DEFAULTS SAVED":"APPLIED AND SAVED"):"active in RAM; SAVE FAILED";
  if (!fault && !imu.fault) reason=saved?(wasFactoryReset?"Factory settings saved":(acceptedTune?"Autotune complete; applied and saved":"Manual settings saved; tuning unverified")):
      "Settings applied in RAM; storage write failed";
  queueTuneReport();
}

void queueStatus() {
  const int n=snprintf(diagnostic,sizeof(diagnostic),
      "ready=%u fault=%u armed=%u R=%.2f P=%.2f gyroR=%.2f gyroP=%.2f gyroValid=%u calibration=%u/600 reason=%s\n",
      imu.ready,fault,armed,imu.attitude.roll(),imu.attitude.pitch(),imu.gyroOnly.roll(),imu.gyroOnly.pitch(),imu.gyroOnlyValid,imu.count,reason);
  diagnosticLength=n>0 ? min((size_t)n,sizeof(diagnostic)-1):0; diagnosticOffset=0;
}
void queueResponseReport() {
  const int n=probe.format(diagnostic,sizeof(diagnostic));
  diagnosticLength=n>0?min((size_t)n,sizeof(diagnostic)-1):0; diagnosticOffset=0;
  phone.publishTuneReport(diagnostic); ++tuneRevision;
}
void queueHoldReport() {
  const int n=holder.format(diagnostic,sizeof(diagnostic));
  diagnosticLength=n>0?min((size_t)n,sizeof(diagnostic)-1):0; diagnosticOffset=0;
  if (holdTraceReady) diagnosticLength+=formatHoldTrace(diagnostic+diagnosticLength,
      sizeof(diagnostic)-diagnosticLength,recorder,postStop);
  phone.publishTuneReport(diagnostic); ++tuneRevision;
}

uint32_t duty(uint16_t us) { return ((uint32_t)us*16384UL+10000UL)/20000UL; }
void resetPids() { rollPid.reset(); pitchPid.reset(); yawPid.reset(); limited=false; controlValid=false; }
void capture(bool final=false) {
  FlightRow row;
  row.roll=imu.attitude.roll(); row.pitch=imu.attitude.pitch();
  row.gyroAngles[0]=imu.gyroOnlyValid?imu.gyroOnly.roll():NAN;
  row.gyroAngles[1]=imu.gyroOnlyValid?imu.gyroOnly.pitch():NAN;
  row.rates[0]=imu.rate.x; row.rates[1]=imu.rate.y; row.rates[2]=imu.rate.z;
  row.integral[0]=rollPid.integral; row.integral[1]=pitchPid.integral; row.integral[2]=yawPid.integral;
  row.integralCap[0]=rollPid.integralLimit; row.integralCap[1]=pitchPid.integralLimit;
  row.g=flight::norm(imu.accel); row.base=lroundf(base); row.target=lroundf(target); row.limited=limited;
  row.requested[0]=bench?0:session.roll;
  row.requested[1]=bench?0:session.pitch;
  row.requested[2]=bench?0:session.yaw;
  if (bench && tuner.running)
    row.requested[tuner.axis==AutoCompare::ROLL?0:1]=AutoCompare::setpoint(millis()-benchStart);
  for (unsigned i=0;i<4;++i) row.sent[i]=sent[i];
  row.calculatedPitch=(virtualUs[0]+virtualUs[1]-virtualUs[2]-virtualUs[3])*0.25f;
  row.pitchTargetRate=pitchPid.lastTarget; row.pitchControlRate=pitchPid.lastRate;
  row.pitchPterm=pitchPid.kp*(pitchPid.lastTarget-pitchPid.lastRate);
  row.pitchDterm=-pitchPid.kd*pitchPid.derivative;
  row.pitchGains[0]=pitchPid.kp; row.pitchGains[1]=pitchPid.ki; row.pitchGains[2]=pitchPid.kd;
  row.angleGain=onlineSettings.angleGain;
  row.balanceTrim[0]=motorCheck?0:flight::balanceTrim(onlineSettings.rollTrim,base);
  row.balanceTrim[1]=motorCheck?0:flight::balanceTrim(onlineSettings.pitchTrim,base);
  row.controlValid=controlValid;
  row.controlAgeMs=(uint16_t)min((uint32_t)65535,(uint32_t)(millis()-lastControlMs));
  recorder.capture(millis(),row,final);
}
StopPose stopPose() {
  return {imu.attitude.roll(),imu.attitude.pitch(),imu.gyroOnly.roll(),imu.gyroOnly.pitch(),
      flight::norm(imu.accel),imu.gyroOnlyValid,imu.rate.y,imu.rate.z};
}
void appendReport(const char *format,...) {
  const size_t room=sizeof(lastReport)-reportLength;
  va_list args; va_start(args,format);
  const int n=vsnprintf(lastReport+reportLength,room,format,args);
  va_end(args);
  if (n>0) reportLength+=min((size_t)n,room-1);
}
void finishPostStop() {
  appendReport("POST-STOP: fresh snapshots; dF=fused change, dG=gyro-only change since STOP (degrees).\n");
  for (size_t i=PostStopTrack::COUNT-3;i<PostStopTrack::COUNT;++i) {
    const auto &s=postStop.samples[i];
    if (!s.valid) { appendReport("OFF +%lu ms: fresh sample unavailable.\n",(unsigned long)s.ms); continue; }
    const auto &p=s.pose; const auto &b=postStop.before.pose;
    appendReport("OFF +%lu ms F=%.2f,%.2f G=%.2f,%.2f dF=%.2f,%.2f dG=%.2f,%.2f g=%.2f valid=%u/%u\n",
        (unsigned long)s.ms,p.roll,p.pitch,p.gyroRoll,p.gyroPitch,
        p.roll-b.roll,p.pitch-b.pitch,p.gyroRoll-b.gyroRoll,p.gyroPitch-b.gyroPitch,p.g,
        postStop.before.valid && b.gyroValid,p.gyroValid);
  }
  appendReport("Gyro-only can drift or suffer vibration; this comparison is diagnostic, not an independent attitude reference.\n");
  phone.publishReport(lastReport);
  if (holdTracePending) { holdTracePending=false; holdTraceReady=true; queueHoldReport(); }
}
void cancelPostStop(const char *why) {
  if (!postStop.cancel()) return;
  holdTracePending=holdTraceReady=false;
  appendReport("POST-STOP canceled: %s.\n",why); phone.publishReport(lastReport);
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
    "StabilizeNow v4 (%s). STOPPED: %s. Motors requested OFF.\nSaved final %u samples at 20 Hz, t=%lu..%lu ms since arm.\n"
    "Roll %.2f..%.2f deg; pitch %.2f..%.2f deg.\n"
    "FINAL BEFORE STOP: R=%.2f P=%.2f rates=%.2f %.2f %.2f g=%.2f\n"
    "Base=%u target=%u; FL0 FR19 RR18 RL3=%u %u %u %u; I=%.2f %.2f %.2f; mix/biasLimit=%u pitchCommand=%.2f us\n"
    "Largest fresh-sample gap=%.2f ms. Pulse requests, not measured thrust.\n",
    lastRunMotorCheck?"3 s equal-pulse check; stabilization disabled":(lastRunWifi?"bounded Wi-Fi trial":"7 s trial"),
    reason,(unsigned)recorder.count,(unsigned long)recorder.at(0).ms,(unsigned long)r.ms,
    recorder.at(ri).roll,recorder.at(ra).roll,recorder.at(pi).pitch,recorder.at(pa).pitch,
    r.roll,r.pitch,r.rates[0],r.rates[1],r.rates[2],r.g,r.base,r.target,
    r.sent[0],r.sent[1],r.sent[2],r.sent[3],r.integral[0],r.integral[1],r.integral[2],r.limited,((float)r.sent[0]+r.sent[1]-r.sent[2]-r.sent[3])*0.25f,maxSampleGapMs);
  reportLength=n>0 ? min((size_t)n,sizeof(lastReport)-1):0;
  appendReport("BALANCE: trim R/P at base 1400=%.1f/%.1f us; scheduled before STOP=%.1f/%.1f us (before limits). Level targets unchanged.\n",
    onlineSettings.rollTrim,onlineSettings.pitchTrim,r.balanceTrim[0],r.balanceTrim[1]);
  const WobbleSummary motion=summarizeWobble(recorder);
  const size_t room=sizeof(lastReport)-reportLength;
  int added=0;
  if (motion.valid) {
    const auto &rAxis=motion.axis[0], &pAxis=motion.axis[1];
    added=snprintf(lastReport+reportLength,room,
      "STEADY WINDOW: %.2f s, %u sampled rows; throttle settled, steering zero.\n"
      "ROLL mean=%.2f deg drift=%.2f deg/s motionRMS=%.2f deg gyroRMS=%.2f deg/s crossingHz=%.2f rises=%u\n"
      "PITCH mean=%.2f deg drift=%.2f deg/s motionRMS=%.2f deg gyroRMS=%.2f deg/s crossingHz=%.2f rises=%u\n"
      "g=%.2f..%.2f; sampled limiter=%.0f%%; integral near cap R=%.0f%% P=%.0f%%.\n"
      "Crossing Hz is a rough motion estimate; 0 means insufficient/irregular crossings, not proof of no wobble. 20 Hz cannot resolve fast vibration.\n",
      motion.seconds,(unsigned)motion.count,
      rAxis.mean,rAxis.slope,rAxis.residualRms,rAxis.rateRms,rAxis.hz,rAxis.rises,
      pAxis.mean,pAxis.slope,pAxis.residualRms,pAxis.rateRms,pAxis.hz,pAxis.rises,
      motion.gMin,motion.gMax,100.0f*motion.limited/motion.count,
      100.0f*rAxis.capped/motion.count,100.0f*pAxis.capped/motion.count);
  } else {
    added=snprintf(lastReport+reportLength,room,"No usable steady window: need >=1 s after throttle settles plus 0.5 s settling; steering must be zero.\n");
  }
  if (added>0) reportLength+=min((size_t)added,room-1);
  appendReport("GYRO WITNESS before STOP: R=%.2f P=%.2f. Gyro-only initialized from fused attitude at ARM.\n",
      r.gyroAngles[0],r.gyroAngles[1]);
  appendReport("Post-stop snapshots follow after 2 s; leave frame untouched. Phone report: refresh after 2 s.\n");
  if (lastRunWifi) appendReport("Wi-Fi test: %lu ms maximum; owner heartbeat required every 500 ms; no repeats.\n",(unsigned long)lastRunDurationMs);
  else if (batch.total) appendReport("USB test %u/%u. Batch repeats require healthy level/still frame and USB; STOP cancels.\n",batch.index,batch.total);
  if (reportLength<sizeof(lastReport)-1)
    reportLength+=formatPitchTrace(lastReport+reportLength,sizeof(lastReport)-reportLength,recorder);
  if (reportLength<sizeof(lastReport)-1)
    reportLength+=formatYawTrace(lastReport+reportLength,sizeof(lastReport)-reportLength,recorder);
  reportOffset=0;
  phone.publishReport(lastReport);
}
void forceLow(unsigned i) {
  if (attached[i]) ledcDetach(PINS[i]);
  attached[i]=false; digitalWrite(PINS[i],LOW); pinMode(PINS[i],OUTPUT); digitalWrite(PINS[i],LOW);
}
void stopAllImpl(const char *why,bool completedBench) {
  const bool hadRun=armed;
  const bool wasWaiting=!armed && (batch.active || tuner.active);
  const bool wasTuning=tuner.active;
  const bool tuningRun=tuner.active && tuner.running;
  const bool probing=probe.active;
  const bool holding=holder.active;
  if (completedBench) batch.complete(millis()); else batch.cancel();
  if (hadRun) capture(true);
  armed=bench=motorCheck=wifiBench=false; session.stop(); base=target=OFF; resetPids();
  if (!fault) reason=why;
  for (unsigned i=0;i<4;++i) {
    virtualUs[i]=OFF;
    if (attached[i] && !ledcWrite(PINS[i],duty(OFF))) {
      forceLow(i); fault=true; reason="OFF write failed; pin detached";
      batch.cancel();
    }
    sent[i]=OFF;
  }
  // Apply/restore settings and format only after every motor OFF write.
  if (hadRun) postStop.begin(millis(),stopPose(),imu.ready && !imu.fault && !fault);
  if (holding) {
    holder.finish(completedBench && !fault && !imu.fault);
    restoreGains(); // The temporary P never survives completion or cancellation.
    batch.cancel();
    holdTracePending=hadRun; holdTraceReady=false;
    if (!fault && completedBench) reason=holder.reason;
    queueHoldReport();
  }
  if (probing) {
    probe.finish(completedBench && !fault && !imu.fault);
    batch.cancel();
    if (!fault && completedBench) reason=probe.reason;
    queueResponseReport();
  }
  if (tuningRun) {
    const bool searchOkay=tuner.finish(completedBench && !fault && !imu.fault);
    if (!searchOkay) batch.cancel();
    batch.total=tuner.planned;
    if (!tuner.active) batch.cancel();
    if (tuner.decision==AutoCompare::REJECTED) {
      if (!fault) reason=tuner.reason;
    } else if (tuner.active) {
      reason=tuner.reason;
    } else if (!tuner.active && (tuner.decision==AutoCompare::RECOMMEND || tuner.decision==AutoCompare::KEEP)) {
      if (settings.acceptVerified(tuner)) {
        acceptedTune=true; factorySavePending=false; savePending=true; storageStatus="applied; save pending";
        reason="Autotune complete; result applied; saving after OFF snapshots";
      } else reason="Search result failed application checks; prior settings retained";
    }
    restoreGains(); queueTuneReport();
  } else if (!completedBench && tuner.active) {
    tuner.cancel(); restoreGains(); queueTuneReport();
  }
  if (hadRun) {
    buildReport(); // All output-stop writes happen before formatting.
  }
  if (wasWaiting && !completedBench) { if (wasTuning) queueTuneReport(); else queueStatus(); }
}
void stopAll(const char *why) { stopAllImpl(why,false); }
void trip(const char *why) {
  // Preserve any previously latched fault.
  if (fault) return;
  fault=true; reason=why; stopAll(why);
}
bool canArm() {
  if (fault || imu.fault || savePending || !imu.ready || (uint32_t)(millis()-bootTime)<5000) return false;
  if (fabsf(imu.attitude.roll())>5 || fabsf(imu.attitude.pitch())>5 || flight::norm(imu.rate)>3) return false;
  for (bool a:attached) if (!a) return false;
  return true;
}
bool beginRun() {
  motorCheck=lastRunMotorCheck=false;
  wifiBench=lastRunWifi=false;lastRunDurationMs=7000;
  cancelPostStop("new ARM before comparison finished");
  holdTracePending=holdTraceReady=false;
  imu.resetGyroWitness();
  if (tuner.active && !tuner.startRun(batch.index)) {
    stopAll("Internal tune sequence check failed; motors remain OFF"); return false;
  }
  restoreGains();
  armed=true; base=target=OFF; resetPids(); maxSampleGapMs=0;
  for (float &v:virtualUs) v=OFF;
  recorder.begin(millis()); reportLength=reportOffset=0;
  reason="armed; zero throttle";
  return true;
}
void startSearch(AutoCompare::Axis axis,int pulse) {
  tuner.begin(axis,settings.roll,settings.pitch);
  if (!tuner.active) { reason=tuner.reason; queueTuneReport(); return; }
  batch.beginSearch(pulse,tuner.planned); usbGuard.reset(); usbPresent=true;
  queueTuneReport(); // Publish the initial snapshot while every motor is still OFF.
  session.stop(); if (!beginRun()) return;
  bench=true; benchStart=millis(); target=pulse;
  reason="Automatic gain search; restrained fixture ONLY";
}
void startResponse(ResponseProbe::Axis axis,int pulse) {
  tuner.cancel(); restoreGains();
  probe.begin(axis); batch.begin(pulse,1); usbGuard.reset(); usbPresent=true;
  session.stop(); if (!beginRun()) return;
  bench=true; benchStart=millis(); target=pulse;
  reason="Single fixture response check; automatic stop after 7 s";
}
void startHold(int pulse) {
  tuner.cancel(); restoreGains();
  holder.begin(settings.pitch.p);
  if (!holder.active) { reason=holder.reason; queueHoldReport(); return; }
  batch.begin(pulse,1); usbGuard.reset(); usbPresent=true;
  session.stop(); if (!beginRun()) return;
  bench=true; benchStart=millis(); target=pulse;
  reason="Temporary lower pitch-P fixture trial; integral enabled; 7 s limit";
}
void acceptPhone() {
  ControlPacket p;
  if (!phone.take(p)) return;
  if (p.action==0) { stopAll("phone STOP"); return; }
  if(wifiBench){
    // Only the owning page's zero-motion heartbeat renews a bounded test.
    if(p.action==2&&p.throttle==1000&&!p.roll&&!p.pitch&&!p.yaw)session.accept(p,millis(),false);
    return;
  }
  if (bench || batch.active || fault || imu.fault) return;
  const auto result=session.accept(p,millis(),canArm() && !armed);
  if (result==ControlSession::BLOCKED) { reason="ARM blocked: need healthy, calibrated, level and still frame at zero throttle"; return; }
  if (result==ControlSession::ARM) { batch.total=0; beginRun(); }
  if (result==ControlSession::UPDATE && armed) {
    target=session.throttle;
    reason=target>=1100 ? "self-level control active":"armed; motors OFF at zero throttle";
  }
}
void publishStatus() {
  char json[PhoneLink::STATUS_SIZE];
  snprintf(json,sizeof(json),
    "{\"ready\":%s,\"fault\":%s,\"armed\":%s,\"canArm\":%s,\"calibration\":%u,\"nonce\":%lu,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"yawRate\":%.2f,\"yawTarget\":%d,\"yawI\":%.2f,\"base\":%u,\"target\":%u,\"motors\":[%u,%u,%u,%u],"
    "\"tuning\":%s,\"trial\":%u,\"trials\":%u,\"trialMs\":%lu,\"tuneRev\":%lu,\"wifiTest\":%s,\"testLimitMs\":%lu,\"setupBusy\":%s,\"reason\":\"%s\"}",
    imu.ready?"true":"false",fault?"true":"false",armed?"true":"false",canArm() && !batch.active?"true":"false",imu.count,
    (unsigned long)session.nonce,imu.attitude.roll(),imu.attitude.pitch(),imu.rate.z,bench?0:session.yaw,yawPid.integral,(unsigned)lroundf(base),(unsigned)lroundf(target),
    sent[0],sent[1],sent[2],sent[3],tuner.active?"true":"false",tuner.run,tuner.planned,
    (unsigned long)(bench && armed?(uint32_t)(millis()-benchStart):0),(unsigned long)tuneRevision,wifiBench?"true":"false",
    (unsigned long)wifiDurationMs,(armed||batch.active||postStop.active||savePending)?"true":"false",reason);
  phone.publish(json);
}
void resetTuning() {
  stopAll("Factory tuning reset"); tuner=AutoCompare{};
  settings=TuneSettings{}; settings.seal();onlineSettings=OnlineSettings{};onlineSettings.seal(); restoreGains();
  // Only an explicit operator reset may persist defaults through a fault.
  // Motor OFF/report gates still apply; neither fault latch is cleared.
  acceptedTune=false; factorySavePending=savePending=true;
  storageStatus="factory reset; save pending";
  queueTuneReport();
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
  if (!strcmp(line,"autotune report")) { if (!armed) queueTuneReport(); return; }
  if (!strcmp(line,"autotune reset")) { resetTuning(); return; }
  if (!strcmp(line,"response report")) { if (!armed) queueResponseReport(); return; }
  if (!strcmp(line,"hold report")) { if (!armed) queueHoldReport(); return; }
  if (!strcmp(line,"cal")) {
    stopAll("recalibrating");
    cancelPostStop("recalibration changes the reference");
    if (!fault) imu.recalibrate();
    return;
  }
  // Fixed equal-pulse diagnostic only; never a stabilized or flight mode.
  if (!strcmp(line,"motorcheck 1250")) {
    if (armed || batch.active || !canArm()) {
      if (!armed) { if (!fault) reason="Motor check blocked: need healthy level/still frame"; queueStatus(); }
      return;
    }
    tuner.cancel(); batch.begin(1250,1); usbGuard.reset(); usbPresent=true;
    session.stop(); if (!beginRun()) return;
    motorCheck=lastRunMotorCheck=bench=true; benchStart=millis(); target=1250;
    reason="PROPS OFF: all four ramp equally to 1250 us; automatic stop after 3 s";
    return;
  }
  int holdPulse=1400;
  if (LevelHold::parse(line,holdPulse)) {
    if (armed || batch.active || !canArm()) {
      if (!armed) { reason="Hold check blocked: need healthy level/still frame"; queueStatus(); }
      return;
    }
    startHold(holdPulse); return;
  }
  ResponseProbe::Axis responseAxis; int responsePulse=1400;
  if (ResponseProbe::parse(line,responseAxis,responsePulse)) {
    if (armed || batch.active || !canArm()) {
      if (!armed) { reason="Response check blocked: need healthy level/still frame"; queueStatus(); }
      return;
    }
    startResponse(responseAxis,responsePulse); return;
  }
  AutoCompare::Axis tuneAxis; int requestedTunePulse=1400;
  if (AutoCompare::parse(line,tuneAxis,requestedTunePulse)) {
    if (armed || batch.active || !canArm()) {
      if (!armed) { reason="Tune blocked: need healthy level/still frame"; queueStatus(); }
      return;
    }
    acceptedTune=false;
    startSearch(tuneAxis,requestedTunePulse); return;
  }
  int pulse=0; unsigned runs=1;
  if (BenchBatch::parse(line,pulse,runs)) {
    if (armed || batch.active || !canArm()) {
      if (!armed) { if (!fault) reason="Test blocked: wait for healthy level calibration"; queueStatus(); }
      return;
    }
    // An explicit count authorizes only these bounded bare-motor runs.
    tuner.cancel();
    batch.begin(pulse,runs);
    usbGuard.reset(); usbPresent=true;
    session.stop(); if (!beginRun()) return;
    bench=true; benchStart=millis(); target=pulse;
    reason="7 s USB bench test; props OFF"; return;
  }
  if (!*line) return;
  stopAll("unknown USB command");
}
void acceptOnline() {
  OnlinePacket p;if(!phone.takeOnline(p))return;
  if(p.nonce!=session.nonce||!p.session||!p.sequence||(uint32_t)(millis()-p.received)>ControlSession::QUEUE_MAX_MS)return;
  if(p.kind<OnlinePacket::APPLY||p.kind>OnlinePacket::HOLD_TEST)return;
  if(p.kind>=OnlinePacket::LEVEL_TEST&&!p.validStart())return;
  // Never defer an armed settings edit or a test start until after STOP.
  if(armed||batch.active||postStop.active||savePending)return;
  onlineAckSession=p.session;onlineAckSequence=p.sequence;
  if(p.kind>=OnlinePacket::LEVEL_TEST){
    if(!canArm()){
      session.stop();onlineResult="Test blocked: wait for READY, level and still";queueOnlineSettings();return;
    }
    ControlPacket arm;arm.nonce=p.nonce;arm.session=p.session;arm.sequence=p.sequence;
    arm.received=p.received;arm.action=1;
    if(session.accept(arm,millis(),true)!=ControlSession::ARM)return;
    tuner.cancel();batch=BenchBatch{};probe=ResponseProbe{};holder=LevelHold{};
    if(p.kind==OnlinePacket::ROLL_RESPONSE||p.kind==OnlinePacket::PITCH_RESPONSE)
      probe.begin(p.kind==OnlinePacket::ROLL_RESPONSE?ResponseProbe::ROLL:ResponseProbe::PITCH);
    if(p.kind==OnlinePacket::HOLD_TEST)holder.begin(settings.pitch.p);
    onlineResult="Starting bounded Wi-Fi test; stay on this page";queueOnlineSettings();
    if(!beginRun())return;
    bench=wifiBench=lastRunWifi=true;benchStart=millis();target=p.pulse;
    wifiDurationMs=lastRunDurationMs=(uint32_t)p.seconds*1000;
    motorCheck=lastRunMotorCheck=p.kind==OnlinePacket::EQUAL_TEST;
    reason=motorCheck?"Wi-Fi equal-pulse check; 3 s; no stabilization":"Bounded Wi-Fi stabilization test; STOP cancels";
    return;
  }
  session.stop(); // A consumed setup nonce cannot be replayed after the action.
  if(p.kind==OnlinePacket::APPLY){
    if(!p.values.valid()){onlineResult="Invalid tuning rejected";queueOnlineSettings();return;}
    onlineSettings=p.values;settings=onlineSettings.pid;useOnlineStore=true;acceptedTune=false;
    restoreGains();resetPids();storageStatus="manual values in RAM; not saved or verified";
    onlineResult="Applied in RAM; manual tuning unverified";queueTuneReport();
  } else if(p.kind==OnlinePacket::SAVE){
    if(fault||imu.fault){onlineResult="Save blocked while faulted";queueOnlineSettings();return;}
    useOnlineStore=true;savePending=true;factorySavePending=false;
    storageStatus="manual save pending; tuning unverified";onlineResult="Save queued; wait for storage confirmation";queueTuneReport();
  } else if(p.kind==OnlinePacket::CALIBRATE){
    if(fault||imu.fault){onlineResult="Sensor fault latched; power cycle needed";queueOnlineSettings();return;}
    cancelPostStop("online recalibration");imu.recalibrate();
    reason="Online calibration: keep frame level and still";onlineResult="Calibrating; wait for READY";queueOnlineSettings();
  } else if(p.kind==OnlinePacket::DEFAULTS){
    onlineResult="Factory reset requested; save pending";resetTuning();
  }
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
  if (batch.active && !usbPresent) { stopAll("USB unavailable >500 ms: batch canceled"); return; }
  if (!armed) return;
  if(wifiBench&&session.expired(millis())){stopAll("Wi-Fi test heartbeat timeout >500 ms");return;}
  if (bench && (uint32_t)(millis()-benchStart)>=(wifiBench?wifiDurationMs:(motorCheck?3000u:BENCH_DURATION_MS)))
    stopAllImpl(wifiBench?"Bounded Wi-Fi test complete":(motorCheck?"3 s equal-pulse motor check complete":"7 s bench test complete"),true);
  else if (!bench && session.expired(millis())) stopAll("phone command timeout >500 ms");
}
void stabilize() {
  if (!armed) return;
  const float roll=imu.attitude.roll(),pitch=imu.attitude.pitch();
  maxSampleGapMs=fmaxf(maxSampleGapMs,imu.dt*1000);
  if (fabsf(roll)>35 || fabsf(pitch)>35) { stopAll("tilt exceeded 35 degrees"); return; }
  if (flight::norm(imu.rate)>120) {
    stopAll("Motion rate exceeded 120 deg/s; manual restart required"); return;
  }
  if ((bench || motorCheck || tuner.active || probe.active || holder.active) && (fabsf(roll)>12 || fabsf(pitch)>12)) {
    stopAll("Automatic test tilt exceeded 12 deg"); return;
  }
  if (target<1100) {
    base=OFF; resetPids(); for (float &v:virtualUs) v=OFF;
    return;
  }
  const float slew=(target>=base ? 200.0f:600.0f)*imu.dt;
  base+=flight::clamp(target-base,-slew,slew);
  if (base<1100) { resetPids(); for (float &v:virtualUs) v=OFF; return; }
  if (motorCheck) {
    resetPids();
    for (float &v:virtualUs) v=base;
    lastControlMs=millis(); controlValid=true;
    return;
  }
  float rollTarget=bench?0:session.roll,pitchTarget=bench?0:session.pitch;
  const uint32_t elapsed=millis()-benchStart;
  if (bench && tuner.running) {
    if (tuner.axis==AutoCompare::ROLL) rollTarget=AutoCompare::setpoint(elapsed);
    else pitchTarget=AutoCompare::setpoint(elapsed);
  }
  const auto m=flight::levelMix(base,roll,pitch,imu.rate,imu.dt,onlineSettings.angleGain,
      rollPid,pitchPid,yawPid,!limited && !probe.active,rollTarget,pitchTarget,bench?0:session.yaw,
      flight::balanceTrim(onlineSettings.rollTrim,base)+(probe.active && probe.axis==ResponseProbe::ROLL?probe.bias(elapsed):0),
      flight::balanceTrim(onlineSettings.pitchTrim,base)+(probe.active && probe.axis==ResponseProbe::PITCH?probe.bias(elapsed):0));
  limited=m.limited;
  lastControlMs=millis(); controlValid=true;
  for (unsigned i=0;i<4;++i) virtualUs[i]=m.pulse[i];
  if (bench && holder.active) {
    const float rc=(m.pulse[0]-m.pulse[1]-m.pulse[2]+m.pulse[3])/4;
    const float pc=(m.pulse[0]+m.pulse[1]-m.pulse[2]-m.pulse[3])/4;
    holder.sample(elapsed,roll,pitch,imu.rate.x,imu.rate.y,rollPid.integral,pitchPid.integral,imu.dt,
        limited || fabsf(rc)>=79 || fabsf(pc)>=79,
        fabsf(rollPid.integral)>=rollPid.integralLimit-0.5f ||
        fabsf(pitchPid.integral)>=pitchPid.integralLimit-0.5f);
  }
  if (bench && probe.active) {
    const bool r=probe.axis==ResponseProbe::ROLL;
    const float correction=r?(m.pulse[0]-m.pulse[1]-m.pulse[2]+m.pulse[3])/4:
        (m.pulse[0]+m.pulse[1]-m.pulse[2]-m.pulse[3])/4;
    probe.sample(elapsed,r?imu.rate.x:imu.rate.y,correction,imu.dt,limited || fabsf(correction)>=79);
  }
  if (bench && tuner.running) {
    const bool r=tuner.axis==AutoCompare::ROLL;
    const float rollCorrection=(m.pulse[0]-m.pulse[1]-m.pulse[2]+m.pulse[3])/4;
    const float pitchCorrection=(m.pulse[0]+m.pulse[1]-m.pulse[2]-m.pulse[3])/4;
    const auto &pid=r?rollPid:pitchPid;
    const bool clipped=limited || fabsf(rollCorrection)>=79 || fabsf(pitchCorrection)>=79;
    tuner.sample(elapsed,roll,pitch,imu.rate.x,imu.rate.y,imu.dt,clipped,
        fabsf(pid.integral)>=pid.integralLimit-0.5f);
  }
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
  loadSettings(); restoreGains();
  networkReady=phone.begin();
  if (!networkReady) { trip("phone network initialization failed"); return; }
  // HTTP formatting/receives run at lower priority than this control task.
  vTaskPrioritySet(nullptr,5);
  bootTime=millis();
  queueTuneReport();
  Serial.printf("StabilizeNow v4 (AUTO PID 7s): time-weighted scoring, drift verification, phone results. Max 16 trials / 112 s powered. DISARMED. Reset=%d.\n",(int)esp_reset_reason());
  Serial.println("Keep frame level and still. Home network: http://quadlevel.local/ . Fallback Wi-Fi QuadLevel / CHANGE_ME_123: http://192.168.4.1/");
  Serial.println("MPU mounting: X arrow FRONT, Y arrow LEFT, Z UP. Body rates: roll right-side down, pitch nose-up, yaw clockwise viewed from above.");
  Serial.println("All armed modes: motion rate >120 deg/s requests motors OFF, no automatic restart. Stopped reports include YAW TIMELINE.");
  Serial.println("Online tuning panel: PID values, calibration, bounded Wi-Fi tests and reports; USB not required after this upload.");
  Serial.println("Online balance trim: +roll boosts LEFT, +pitch boosts FRONT; negatives boost RIGHT/REAR. Defaults zero; limits unchanged.");
  if(WiFi.status()==WL_CONNECTED) Serial.printf("Home control address: http://%s/\n",WiFi.localIP().toString().c_str());
  else Serial.println("Home Wi-Fi unavailable at startup; use the QuadLevel fallback network.");
  Serial.println("USB props-off: test 1400 or test 1400 3. stop cancels batch. status | report | cal");
  Serial.println("USB PROPS OFF ONLY: motorcheck 1250. Equal pulses, no stabilization, 3 s including ramp, no repeats. stop cancels.");
  Serial.println("Fixture ONLY: autotune 1400 runs BOTH axes automatically. 7 s trials, >=5 s OFF. STOP cancels all.");
  Serial.println("Optional: autotune roll 1400 | autotune pitch 1400 (8 trials). autotune report | autotune reset");
  Serial.println("Fixture response check: response pitch 1400 | response roll 1400. Single 7 s run; response report repeats result.");
  Serial.println("Fixture trial: hold 1400. Temporary pitch P x0.5 (floor 0.25), I/D unchanged, 7 s then restore. Automatic time trace after 2 s OFF; hold report reprints.");
  Serial.printf("Active R P/I/D=%.3f/%.3f/%.4f; P P/I/D=%.3f/%.3f/%.4f; %s.\n",
      settings.roll.p,settings.roll.i,settings.roll.d,settings.pitch.p,settings.pitch.i,settings.pitch.d,storageStatus);
  if (!imu.begin()) { trip(imu.fault); queueStatus(); }
}
void loop() {
  usbRawConnected=(bool)Serial;
  usbPresent=usbGuard.observe(millis(),usbRawConnected);
  enforceLink(); readSerial(); acceptPhone(); acceptOnline(); enforceLink();
  bool fresh=false;
  if (!fault) {
    const bool wasReady=imu.ready;
    fresh=imu.tick();
    if (imu.fault) trip(imu.fault);
    else if (!wasReady && imu.ready) { reason="READY; calibrated level reference; motors OFF"; queueStatus(); }
    if (fresh && !fault) stabilize();
    enforceLink();
    if (!fault) writeOutputs();
    if (fresh && armed && !fault) capture();
  }
  if (!armed && postStop.active && postStop.tick(millis(),fresh,imu.ready && !fault && !imu.fault,stopPose()))
    finishPostStop();
  if (!armed && batch.active) {
    const auto next=batch.poll(millis(),usbPresent,canArm(),
        usbRawConnected && !postStop.active && reportOffset>=reportLength && diagnosticOffset>=diagnosticLength);
    if (next==BenchBatch::START) {
      if (beginRun()) {
        bench=true; benchStart=millis(); target=batch.pulse;
        reason=tuner.active?"Automatic next PID trial; restrained fixture ONLY":"7 s USB batch test; props OFF";
      }
    } else if (next==BenchBatch::CANCELED) {
      tuner.cancel(); restoreGains();
      reason="Batch canceled: need healthy level/still frame, USB and completed report";
      queueTuneReport();
    }
  }
  processSettingsSave();
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
