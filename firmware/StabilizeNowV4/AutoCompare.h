#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// Bounded coordinate search of the actual rate PID gains. This measures fixture
// response, not flight or position hold. Identical ramps/commands and fresh PID
// history make short trials comparable; final repeats can still reject them.
class AutoCompare {
 public:
  enum Axis { ROLL, PITCH, BOTH };
  enum Parameter { P, D, I, NONE_PARAMETER };
  enum Role { BASE, LOWER, HIGHER, VERIFY };
  enum Decision { NONE, TESTING, REJECTED, KEEP, RECOMMEND };
  enum Failure { RESPONSE=1, INTEGRAL_LIMIT=2, DATA=4, NOISY=8,
                 INTERRUPTED=16, REVERSED=32, OUTPUT_LIMIT=64, QUALITY=128 };
  static constexpr unsigned MAX_RUNS=16;
  static constexpr uint32_t DURATION_MS=7000;
  struct Gains { float p=0.5f,i=0.12f,d=0.002f; };
  struct Result {
    float error=0,rate=0,tail=0,tailMean=0,tailRate=0,otherTail=0,otherTailMean=0,otherTailRate=0;
    float score=0,span=0,rotation[2]={};
    float tailSlope=0,otherTailSlope=0;
    unsigned failure=0;
    bool done=false,valid=false,selected=false;
  };
  struct Trial {
    Axis axis=ROLL; Parameter parameter=NONE_PARAMETER; Role role=BASE;
    Gains gains[2]; Result result;
  };
  bool active=false,running=false;
  Axis axis=ROLL,scope=BOTH;
  Decision decision=NONE;
  unsigned run=0,planned=0,failure=0;
  Trial trials[MAX_RUNS];
  const char *reason="No PID search recorded";

  static bool parse(const char *line,Axis &selected,int &pulse) {
    char name[8]={}; int us=1400,n=-1; selected=BOTH;
    if (strncmp(line,"autotune",8) || (line[8] && line[8]!=' ')) return false;
    if (!strcmp(line,"autotune")) { pulse=us; return true; }
    if (sscanf(line,"autotune %4d%n",&us,&n)==1 && n==(int)strlen(line)) {
      if (us<1250 || us>1400) return false;
      pulse=us; return true;
    }
    n=-1; us=1400;
    if (sscanf(line,"autotune %7s %4d%n",name,&us,&n)!=2 || n!=(int)strlen(line)) {
      n=-1; us=1400;
      if (sscanf(line,"autotune %7s%n",name,&n)!=1 || n!=(int)strlen(line)) return false;
    }
    if (us<1250 || us>1400) return false;
    if (!strcmp(name,"roll")) selected=ROLL;
    else if (!strcmp(name,"pitch")) selected=PITCH;
    else if (!strcmp(name,"all")) selected=BOTH;
    else return false;
    pulse=us; return true;
  }
  static float lower(Parameter p) { return p==P?0.25f:(p==I?0.02f:0.0005f); }
  static float upper(Parameter p) { return p==P?1.20f:(p==I?0.30f:0.006f); }
  static bool validGains(const Gains &g) {
    return isfinite(g.p) && isfinite(g.i) && isfinite(g.d) &&
        g.p>=lower(P) && g.p<=upper(P) && g.i>=lower(I) && g.i<=upper(I) &&
        g.d>=lower(D) && g.d<=upper(D);
  }
  void begin(Axis selected,Gains roll,Gains pitch) {
    *this=AutoCompare{}; scope=selected;
    initial[0]=working[0]=roll; initial[1]=working[1]=pitch;
    if ((selected!=ROLL && selected!=PITCH && selected!=BOTH) ||
        !validGains(roll) || !validGains(pitch)) {
      decision=REJECTED; reason="Starting PID settings outside supported bounds"; return;
    }
    const Parameter order[]={P,D,I}; // Response, damping, then steady correction.
    for (unsigned a=0;a<2;++a) if (selected==BOTH || selected==(Axis)a) {
      add((Axis)a,BASE,NONE_PARAMETER);
      for (Parameter p:order) { add((Axis)a,LOWER,p); add((Axis)a,HIGHER,p); }
    }
    for (unsigned a=0;a<2;++a) if (selected==BOTH || selected==(Axis)a)
      add((Axis)a,VERIFY,NONE_PARAMETER);
    active=true; decision=TESTING; reason="Starting seven-second PID search";
  }
  bool startRun(unsigned number) {
    if (!active || running || number!=run+1 || number>planned || number>MAX_RUNS) return false;
    run=number; auto &t=trials[run-1]; axis=t.axis;
    if (t.role==LOWER) for (unsigned a=0;a<2;++a) pairCenter[a]=working[a];
    for (unsigned a=0;a<2;++a) t.gains[a]=t.role==LOWER || t.role==HIGHER?pairCenter[a]:working[a];
    if (t.role==LOWER || t.role==HIGHER) {
      float &v=coefficient(t.gains[axis],t.parameter);
      v=clamp(v*(t.role==LOWER?0.65f:1.50f),lower(t.parameter),upper(t.parameter));
    }
    if (!validGains(t.gains[0]) || !validGains(t.gains[1]))
      return reject("Internal candidate bounds check failed");
    samples=Accumulator{}; failure=0; running=true; return true;
  }
  Gains gains(Axis a) const {
    return running && run>=1 && run<=MAX_RUNS?trials[run-1].gains[a]:working[a];
  }
  Gains selectedGains(Axis a) const { return working[a]; }
  bool verified() const {
    return verifiedSelection && !active && !running && run==planned &&
        (decision==KEEP || decision==RECOMMEND);
  }
  static float setpoint(uint32_t elapsed) {
    if (elapsed>=2500 && elapsed<3500) return 3;
    if (elapsed>=4250 && elapsed<5250) return -3;
    return 0; // Last 1.75 s measures return to level and residual lean.
  }
  void sample(uint32_t elapsed,float roll,float pitch,float rollRate,float pitchRate,
              float dt,bool outputLimited,bool integralLimited) {
    if (!active || !running || elapsed<2500 || elapsed>=DURATION_MS) return;
    if (!isfinite(roll) || !isfinite(pitch) || !isfinite(rollRate) || !isfinite(pitchRate) ||
        !isfinite(dt) || dt<=0 || dt>0.015f) { samples.bad=true; return; }
    if (samples.haveTimestamp && elapsed<=samples.lastElapsed) { samples.bad=true; return; }
    samples.haveTimestamp=true; samples.lastElapsed=elapsed;
    const float angle=axis==ROLL?roll:pitch,rate=axis==ROLL?rollRate:pitchRate;
    const float other=axis==ROLL?pitch:roll,otherRate=axis==ROLL?pitchRate:rollRate;
    const float e=setpoint(elapsed)-angle;
    ++samples.count; samples.seconds+=dt;
    // Integrate cost over elapsed time, not sample count: clustered samples
    // must not make a noisy or clipped interval seem better or worse.
    samples.error2+=e*e*dt; samples.rate2+=rate*rate*dt;
    samples.other2+=other*other*dt; samples.otherRate2+=otherRate*otherRate*dt;
    if (outputLimited) samples.outputSeconds+=dt;
    if (integralLimited) samples.integralSeconds+=dt;
    const int segment=elapsed<3500?0:(elapsed>=4250 && elapsed<5250?1:-1);
    if (segment>=0) samples.rotation[segment]+=rate*dt;
    const int window=elapsed>=3250 && elapsed<3500?0:(elapsed>=5000 && elapsed<5250?1:-1);
    if (window>=0) { samples.angle[window]+=angle*dt; samples.windowSeconds[window]+=dt; ++samples.rows[window]; }
    if (elapsed>=6000) {
      ++samples.tailCount; samples.tailSeconds+=dt;
      samples.tailSum+=angle*dt; samples.tail2+=angle*angle*dt;
      samples.tailRate2+=rate*rate*dt; samples.otherTail2+=other*other*dt;
      samples.otherTailSum+=other*dt; samples.otherTailRate2+=otherRate*otherRate*dt;
      const float t=(elapsed-6000)*0.001f;
      samples.tailT+=t*dt; samples.tailTT+=t*t*dt;
      samples.tailTA+=t*angle*dt; samples.otherTailTA+=t*other*dt;
    }
  }
  // Weak/integral-limited candidates do not cancel the remaining comparisons.
  // Safety/data faults and failed final verification still stop the schedule.
  bool finish(bool completed) {
    if (!active || !running) return false;
    running=false; auto &t=trials[run-1]; auto &r=t.result; r.done=true;
    if (samples.seconds>0) {
      r.error=sqrtf(samples.error2/samples.seconds); r.rate=sqrtf(samples.rate2/samples.seconds);
      r.score=r.error+0.04f*r.rate+0.15f*sqrtf(samples.other2/samples.seconds)+
          0.01f*sqrtf(samples.otherRate2/samples.seconds);
    }
    const float trendDenom=samples.tailSeconds*samples.tailTT-samples.tailT*samples.tailT;
    if (samples.tailSeconds>0) {
      r.tail=sqrtf(samples.tail2/samples.tailSeconds); r.tailMean=samples.tailSum/samples.tailSeconds;
      r.tailRate=sqrtf(samples.tailRate2/samples.tailSeconds);
      r.otherTail=sqrtf(samples.otherTail2/samples.tailSeconds);
      r.otherTailMean=samples.otherTailSum/samples.tailSeconds;
      r.otherTailRate=sqrtf(samples.otherTailRate2/samples.tailSeconds);
      if (trendDenom>0.000001f) {
        r.tailSlope=(samples.tailSeconds*samples.tailTA-samples.tailT*samples.tailSum)/trendDenom;
        r.otherTailSlope=(samples.tailSeconds*samples.otherTailTA-samples.tailT*samples.otherTailSum)/trendDenom;
      }
      r.score+=0.5f*r.tail+0.15f*fabsf(r.tailSlope)+0.05f*fabsf(r.otherTailSlope);
    }
    if (samples.windowSeconds[0]>0 && samples.windowSeconds[1]>0)
      r.span=samples.angle[0]/samples.windowSeconds[0]-samples.angle[1]/samples.windowSeconds[1];
    for (unsigned j=0;j<2;++j) r.rotation[j]=samples.rotation[j];
    failure=0;
    if (!completed) failure|=INTERRUPTED;
    if (samples.bad || samples.count<700 || samples.seconds<4.1f || samples.seconds>4.9f ||
        samples.rows[0]<25 || samples.rows[1]<25 || samples.tailCount<120 ||
        samples.windowSeconds[0]<0.15f || samples.windowSeconds[1]<0.15f ||
        samples.tailSeconds<0.85f || samples.tailSeconds>1.15f || trendDenom<=0.000001f ||
        !isfinite(r.score)) failure|=DATA;
    if (!(r.rotation[0]>0.5f && r.rotation[1]<-0.5f && r.span>=1.5f)) failure|=RESPONSE;
    if (r.rotation[0]<-0.5f && r.rotation[1]>0.5f && r.span<=-1.5f) failure|=REVERSED;
    if (r.rate>=40 || (samples.seconds>0 && sqrtf(samples.otherRate2/samples.seconds)>=40)) failure|=NOISY;
    if (samples.outputSeconds>0.01f*samples.seconds) failure|=OUTPUT_LIMIT;
    if (samples.integralSeconds>0.01f*samples.seconds) failure|=INTEGRAL_LIMIT;
    r.failure=failure; r.valid=failure==0;
    const unsigned fatal=DATA|NOISY|INTERRUPTED|REVERSED|OUTPUT_LIMIT;
    if (failure&fatal) {
      const char *why=failure&INTERRUPTED?"Canceled/interrupted; prior settings retained":
          (failure&DATA?"Faulty timing or insufficient fresh samples; prior settings retained":
          (failure&REVERSED?"Reversed tilt response; prior settings retained":
          (failure&NOISY?"Excessive gyro motion; prior settings retained":"Output correction saturated; prior settings retained")));
      return reject(why);
    }
    if (t.role==VERIFY) {
      if (!r.valid || best[axis]<0 || !consistent(trials[best[axis]].result,r) ||
          r.tail>2.5f || fabsf(r.tailMean)>2.0f || r.tailRate>12 || fabsf(r.tailSlope)>1.5f)
        return rejectVerification(r,"Final repeat/level check failed; no new settings saved");
      if (r.otherTail>2.5f || fabsf(r.otherTailMean)>2.0f || r.otherTailRate>12 || fabsf(r.otherTailSlope)>1.5f)
        return rejectVerification(r,"Final check: other axis still leaning/moving; no new settings saved");
      const auto &original=trials[reference[axis]].result;
      if (axisChanged(axis) && original.valid &&
          fmaxf(r.score,trials[best[axis]].result.score)>=0.90f*original.score)
        return rejectVerification(r,"Measured improvement did not survive repeat check; no new settings saved");
      confirmed[axis]=true;
      if (run<planned) { reason="First axis verified; automatically checking the other axis"; return true; }
      for (unsigned a=0;a<2;++a) if ((scope==BOTH || scope==(Axis)a) && !confirmed[a])
        return rejectVerification(r,"Final verification incomplete; no new settings saved");
      active=false; verifiedSelection=true;
      decision=changed()?RECOMMEND:KEEP;
      reason=decision==RECOMMEND?"PID search verified; apply and save selected gains":
          "PID search verified; no sufficiently large improvement over starting gains";
      return true;
    }
    if (t.role==BASE) {
      reference[axis]=(int)run-1;
      if (r.valid) { best[axis]=(int)run-1; r.selected=true; }
    } else if (t.role==HIGHER) {
      // Rank this pair first. Applying hysteresis after each candidate biases
      // the result toward whichever improving candidate was visited first.
      int pairWinner=-1;
      for (unsigned j=run-2;j<run;++j) {
        const auto &candidate=trials[j].result;
        if (candidate.valid && (pairWinner<0 || candidate.score<trials[pairWinner].result.score))
          pairWinner=(int)j;
      }
      int winner=best[axis];
      if (pairWinner>=0 && (winner<0 || trials[pairWinner].result.score<trials[winner].result.score-
          fmaxf(0.03f,0.05f*trials[winner].result.score))) winner=pairWinner;
      if (winner!=best[axis]) {
        if (best[axis]>=0) trials[best[axis]].result.selected=false;
        best[axis]=winner; trials[winner].result.selected=true;
        working[axis]=trials[winner].gains[axis];
      }
      if (t.parameter==I && best[axis]<0)
        return reject(axis==ROLL?"Roll P/D/I candidates exhausted: no usable response; nothing saved":
            "Pitch P/D/I candidates exhausted: no usable response; nothing saved");
    }
    reason=r.valid?"Trial measured; automatically continuing after OFF gap":
        "Candidate rejected; automatically trying next setting after OFF gap";
    return true;
  }
  void cancel() {
    if (!active) return;
    if (running) finish(false);
    reject("STOP/cancellation: prior settings retained");
  }
  int format(char *out,size_t size) const {
    if (size) out[0]=0;
    size_t used=0;
    if (decision==NONE) { append(out,size,used,"No automatic PID search recorded in this boot.\n"); return (int)used; }
    append(out,size,used,"AUTO PID 7s: %s\nTrial %u/%u. Time-weighted score; 7 s powered / >=5 s OFF. P/D/I comparisons.\n",
        reason,run,planned);
    for (unsigned j=0;j<run && j<MAX_RUNS;++j) {
      const auto &t=trials[j]; const auto &r=t.result;
      if (!r.done) continue;
      append(out,size,used,"#%u %s %s%s P/I/D=%.3f/%.3f/%.4f E=%.2f tail=%.2f drift=%.2f G=%.1f score=%.2f %s%s\n",
          j+1,t.axis==ROLL?"R":"P",t.role==BASE?"BASE":(t.role==VERIFY?"VERIFY":parameterName(t.parameter)),
          t.role==LOWER?"-":(t.role==HIGHER?"+":""),t.gains[t.axis].p,t.gains[t.axis].i,t.gains[t.axis].d,
          r.error,r.tail,r.tailSlope,r.rate,r.score,r.valid?"PASS":"REJECT",r.selected?" SELECTED":"");
    }
    if (run && run<=MAX_RUNS) {
      const auto &r=trials[run-1].result;
      if (r.done && !r.valid) append(out,size,used,
          "Last failure mask=%u [response=1 Icap=2 data=4 noisy=8 stop=16 reversed=32 output=64 quality=128].\n"
          "Gyro response +3/-3=%.2f/%.2f deg (need >0.50/<-0.50); span=%.2f (need >=1.50).\n",
          r.failure,r.rotation[0],r.rotation[1],r.span);
    }
    if (run && run<=MAX_RUNS && (trials[run-1].result.failure&QUALITY)) {
      const auto &r=trials[run-1].result;
      append(out,size,used,"Final RMS/mean/rate/drift: tested=%.2f/%.2f/%.2f/%.2f other=%.2f/%.2f/%.2f/%.2f; limits 2.5 deg / abs 2 deg / 12 deg/s / abs 1.5 deg/s.\n",
          r.tail,r.tailMean,r.tailRate,r.tailSlope,r.otherTail,r.otherTailMean,r.otherTailRate,r.otherTailSlope);
    }
    append(out,size,used,"%s R P/I/D=%.3f/%.3f/%.4f; P P/I/D=%.3f/%.3f/%.4f\n",
        verified()?"VERIFIED BENCH SELECTION":"PROVISIONAL ONLY; NOT SAVED",
        working[0].p,working[0].i,working[0].d,working[1].p,working[1].i,working[1].d);
    append(out,size,used,"Best among measured candidates; short fixture tests do not prove flight stability or battery health.\n");
    return (int)used;
  }
 private:
  Gains initial[2],working[2],pairCenter[2];
  int best[2]={-1,-1};
  int reference[2]={-1,-1};
  bool confirmed[2]={},verifiedSelection=false;
  struct Accumulator {
    unsigned count=0,rows[2]={},tailCount=0;
    float seconds=0,error2=0,rate2=0,other2=0,otherRate2=0;
    float outputSeconds=0,integralSeconds=0,windowSeconds[2]={};
    float angle[2]={},rotation[2]={},tailSum=0,tail2=0,tailRate2=0,otherTail2=0;
    float otherTailSum=0,otherTailRate2=0,tailSeconds=0;
    float tailT=0,tailTT=0,tailTA=0,otherTailTA=0;
    uint32_t lastElapsed=0;
    bool bad=false,haveTimestamp=false;
  } samples;
  void add(Axis a,Role role,Parameter p) {
    if (planned>=MAX_RUNS) return;
    auto &t=trials[planned++]; t.axis=a; t.role=role; t.parameter=p;
  }
  bool reject(const char *why) {
    active=running=verifiedSelection=false; decision=REJECTED; reason=why; return false;
  }
  bool rejectVerification(Result &r,const char *why) {
    r.failure|=QUALITY; r.valid=false; failure=r.failure;
    return reject(why);
  }
  bool changed() const {
    for (unsigned a=0;a<2;++a) if (axisChanged(a)) return true;
    return false;
  }
  bool axisChanged(unsigned a) const {
    return fabsf(initial[a].p-working[a].p)>0.00001f ||
        fabsf(initial[a].i-working[a].i)>0.00001f || fabsf(initial[a].d-working[a].d)>0.000001f;
  }
  static bool consistent(const Result &a,const Result &b) {
    return a.valid && b.valid && fabsf(a.score-b.score)<=0.25f*fmaxf(0.2f,0.5f*(a.score+b.score)) &&
        fabsf(a.span-b.span)<=0.25f*fmaxf(1.5f,0.5f*(a.span+b.span));
  }
  static float clamp(float v,float lo,float hi) { return v<lo?lo:(v>hi?hi:v); }
  static float &coefficient(Gains &g,Parameter p) { return p==P?g.p:(p==I?g.i:g.d); }
  static const char *parameterName(Parameter p) { return p==P?"P":(p==I?"I":"D"); }
  static void append(char *out,size_t size,size_t &used,const char *fmt,...) {
    if (!size || used>=size-1) return;
    va_list args; va_start(args,fmt);
    const int n=vsnprintf(out+used,size-used,fmt,args); va_end(args);
    if (n>0) used+=(size_t)n<size-used?(size_t)n:size-used-1;
  }
};
