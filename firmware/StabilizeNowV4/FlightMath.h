#pragma once
#include <math.h>
#include <stdint.h>

namespace flight {
constexpr float kPi = 3.14159265359f;
constexpr float kRollKp = 0.50f; // Controlled roll-P trial; V3 used 0.70.
constexpr float kBalanceKi = 0.12f; // Candidate bench tuning, not flight-validated.
constexpr float kPitchBalanceKi = 0.12f;
// Candidate rate-bias rejection, not absolute heading hold or a verified tune.
constexpr float kYawBalanceKi = 0.12f;
constexpr float kYawIntegralLimit = 15.0f;
constexpr float kIntegralMinBase = 1250.0f;
inline float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
struct Vec { float x, y, z; };
inline float norm(Vec v) { return sqrtf(v.x*v.x+v.y*v.y+v.z*v.z); }
// Sensor +X forward, +Y left with its +Z face up; body is front/right/down.
// Face-down mounting keeps +X forward and reverses the other two sensor axes.
inline Vec sensorToBody(Vec v,int face) { return {v.x,-face*v.y,-face*v.z}; }
struct Quaternion {
  float w=1, x=0, y=0, z=0;
  static Quaternion fromGravity(Vec g) {
    const float r=atan2f(g.y, g.z), p=atan2f(-g.x, sqrtf(g.y*g.y+g.z*g.z));
    const float cr=cosf(r/2), sr=sinf(r/2), cp=cosf(p/2), sp=sinf(p/2);
    return {cr*cp, sr*cp, cr*sp, -sr*sp};
  }
  Vec rotate(Vec v) const {
    const Vec t={2*(y*v.z-z*v.y), 2*(z*v.x-x*v.z), 2*(x*v.y-y*v.x)};
    return {v.x+w*t.x+y*t.z-z*t.y, v.y+w*t.y+z*t.x-x*t.z, v.z+w*t.z+x*t.y-y*t.x};
  }
  Vec gravity() const { return {2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)}; }
  bool normalize() {
    const float n=sqrtf(w*w+x*x+y*y+z*z);
    if (!isfinite(n) || n<0.5f) return false;
    w/=n; x/=n; y/=n; z/=n; return true;
  }
  bool update(Vec accel, Vec gyro, float dt) {
    // Body axes: X front, Y right, Z down. Rest acceleration is (0,0,-1g).
    Vec omega={gyro.x*kPi/180, gyro.y*kPi/180, gyro.z*kPi/180};
    const float an=norm(accel);
    if (an>0.8f && an<1.2f) {
      const Vec a={-accel.x/an, -accel.y/an, -accel.z/an}, v=gravity();
      omega.x+=2*(a.y*v.z-a.z*v.y);
      omega.y+=2*(a.z*v.x-a.x*v.z);
      omega.z+=2*(a.x*v.y-a.y*v.x);
    }
    const Quaternion old=*this;
    w+=(-old.x*omega.x-old.y*omega.y-old.z*omega.z)*dt/2;
    x+=(old.w*omega.x+old.y*omega.z-old.z*omega.y)*dt/2;
    y+=(old.w*omega.y-old.x*omega.z+old.z*omega.x)*dt/2;
    z+=(old.w*omega.z+old.x*omega.y-old.y*omega.x)*dt/2;
    return normalize();
  }
  float roll() const { return atan2f(2*(w*x+y*z), 1-2*(x*x+y*y))*180/kPi; }
  float pitch() const { return asinf(clamp(2*(w*y-z*x), -1, 1))*180/kPi; }
};

struct RatePid {
  float kp, ki, kd;
  float integralLimit=25;
  float integral=0, lastRate=0, derivative=0;
  bool initialized=false;
  float lastTarget=0; // Diagnostic: target used by the last completed rate step.
  void reset() { integral=lastRate=derivative=lastTarget=0; initialized=false; }
  float step(float target, float measured, float dt, bool integrate) {
    lastTarget=target;
    const float error=target-measured;
    const float raw=initialized ? (measured-lastRate)/dt : 0;
    const float alpha=dt/(1/(2*kPi*15)+dt);
    derivative+=alpha*(raw-derivative);
    lastRate=measured; initialized=true;
    // D on measurement avoids derivative kick on a new setpoint.
    const float pd=kp*error-kd*derivative;
    if (integrate) {
      const float bound=clamp(integralLimit,0,60);
      const float candidate=clamp(integral+ki*error*dt,-bound,bound);
      const float output=pd+candidate;
      // Freeze growth into this axis's limit, while allowing unwind.
      // Mixer saturation is separately passed in through 'integrate'.
      if ((output>=-80 && output<=80) || (output>80 && error<0) || (output<-80 && error>0))
        integral=candidate;
    }
    return clamp(pd+integral, -80, 80);
  }
};

struct Mix { float pulse[4]; bool limited; };
// A bounded manual balance correction, faded with command power to avoid a
// sudden offset at motor startup. This is not a measured thrust/CoM model.
inline float balanceTrim(float at1400,float base) {
  return at1400*clamp((base-1100)/300,0,1);
}
inline Mix mix(float base, float r, float p, float y) {
  // C6: FL0 CCW, FR19 CW, RR18 CCW, RL3 CW, all viewed from above.
  // FRD: positive roll -> left thrust up; positive pitch (nose up) -> front thrust up.
  // CCW rotors produce positive (clockwise) body yaw reaction.
  const float d[4]={r+p+y, -r+p-y, -r-p+y, r-p-y};
  float scale=1;
  for (float v : d) {
    if (v>0) scale=fminf(scale, (1800-base)/v);
    if (v<0) scale=fminf(scale, (base-1100)/-v);
  }
  scale=clamp(scale, 0, 1);
  Mix result={{}, scale<0.9999f};
  for (unsigned i=0; i<4; ++i) result.pulse[i]=clamp(base+scale*d[i], 1100, 1800);
  return result;
}

inline Mix levelMix(float base,float roll,float pitch,Vec rate,float dt,float angleGain,
                    RatePid &rollPid,RatePid &pitchPid,RatePid &yawPid,bool integrate,
                    float rollTarget=0,float pitchTarget=0,float yawTarget=0,
                    float rollBias=0,float pitchBias=0) {
  // Do not learn an imbalance while motors are starting or nearly OFF.
  if (base<kIntegralMinBase) rollPid.integral=pitchPid.integral=yawPid.integral=0;
  integrate=integrate && base>=kIntegralMinBase;
  const float r=rollPid.step(clamp(angleGain*(rollTarget-roll),-80,80),rate.x,dt,integrate);
  const float p=pitchPid.step(clamp(angleGain*(pitchTarget-pitch),-80,80),rate.y,dt,integrate);
  const float y=yawPid.step(yawTarget,rate.z,dt,integrate);
  // Balance feedforward and optional fixture probe share existing axis bounds.
  const float rb=r+rollBias,pb=p+pitchBias;
  Mix result=mix(base,clamp(rb,-80,80),clamp(pb,-80,80),y);
  result.limited=result.limited || fabsf(rb)>80 || fabsf(pb)>80;
  return result;
}

// Runs before attaching motor outputs. Test actual controller math, not a copy.
inline bool selfTest() {
  const Vec forward=sensorToBody({1,0,0},1),left=sensorToBody({0,1,0},1),up=sensorToBody({0,0,1},1);
  if (forward.x!=1 || forward.y!=0 || forward.z!=0 ||
      left.x!=0 || left.y!=-1 || left.z!=0 || up.x!=0 || up.y!=0 || up.z!=-1) return false;
  RatePid saturationProbe{1,1,0};
  for (unsigned i=0;i<100;++i) saturationProbe.step(200,0,0.005f,true);
  if (fabsf(saturationProbe.integral)>0.0001f) return false;
  saturationProbe.step(10,0,0.005f,true);
  if (saturationProbe.integral<=0) return false;
  const Mix r=mix(1400, 40, 0, 0), p=mix(1400, 0, 40, 0), y=mix(1400, 0, 0, 40);
  if (!(r.pulse[0]>r.pulse[1] && r.pulse[3]>r.pulse[2] &&
        p.pulse[0]>p.pulse[3] && p.pulse[1]>p.pulse[2] &&
        y.pulse[0]>y.pulse[1] && y.pulse[2]>y.pulse[3])) return false;
  // Physical oracle: moment = position cross force. Rotor thrust is along -Z.
  const float px[4]={1,1,-1,-1}, py[4]={-1,1,1,-1};
  float mx=0,my=0;
  for (unsigned i=0;i<4;++i) {
    mx-=py[i]*(r.pulse[i]-1400);
    my+=px[i]*(p.pulse[i]-1400);
  }
  if (mx<=0 || my<=0) return false;
  const float bases[3]={1100,1400,1700}, rollCases[3]={-80,0,80};
  for (float base : bases) {
    for (float rr : rollCases) {
      const Mix m=mix(base, rr, 80, -80);
      float mean=0;
      for (float value : m.pulse) { if (value<1100 || value>1800) return false; mean+=value/4; }
      if (fabsf(mean-base)>0.01f) return false;
    }
  }
  const Quaternion mount=Quaternion::fromGravity({0.15f,-0.2f,0.9682458f});
  const Vec aligned=mount.rotate({0.15f,-0.2f,0.9682458f});
  if (fabsf(aligned.x)>0.001f || fabsf(aligned.y)>0.001f || aligned.z<0.99f) return false;
  Quaternion q;
  q.x=sinf(10*kPi/360); q.w=cosf(10*kPi/360);
  for (unsigned i=0; i<1000; ++i) if (!q.update({0,0,-1},{0,0,0},0.005f)) return false;
  if (fabsf(q.roll())>0.05f) return false;
  Quaternion yaw;
  for (unsigned i=0; i<200; ++i) if (!yaw.update({0,0,-1},{0,0,90},0.005f)) return false;
  if (fabsf(yaw.roll())>0.01f || fabsf(yaw.pitch())>0.01f || fabsf(yaw.z-sinf(kPi/4))>0.001f) return false;
  return true;
}
} // namespace flight
