#pragma once
static const char PHONE_PAGE[] PROGMEM=R"PAGE(
<!doctype html><html lang="en"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Quad Level Test</title><style>
body{font:17px system-ui;background:#111a24;color:#edf3fa;max-width:540px;margin:20px auto;padding:0 16px}h1{font-size:25px}button{font:inherit;border:0;border-radius:10px;padding:16px;background:#31465b;color:white;touch-action:none}button:disabled{opacity:.4}.stop{background:#c32335;width:100%;font-weight:800;font-size:24px;margin:14px 0}.arm{background:#187653;width:100%}input[type=range]{width:100%;height:44px}pre{white-space:pre-wrap;background:#1e2d3d;padding:12px;border-radius:8px;font-size:14px}.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:16px 0}.small{font-size:14px;color:#b8c8d8}a{color:#a7d4ff}
</style><h1>Quad Level Test</h1>
<p class="small">Experimental self-level controller. Calibrate level and still. First checks: remove all props. No automatic takeoff, height or position hold.</p>
<pre id="status">Connecting...</pre>
<label><input id="enable" type="checkbox"> Enable physical motor outputs</label>
<button id="arm" class="arm" disabled>ARM at zero throttle</button>
<button id="stop" class="stop">STOP MOTORS</button>
<label>Throttle: <b id="throttleText">1000</b> &micro;s</label><input id="throttle" type="range" min="1000" max="1700" step="5" value="1000" disabled>
<p class="small">The controller varies all four motors to follow the requested tilt. Release steering to request level. Phone must remain on this page.</p>
<div class="pad"><span></span><button data-axis="pitch" data-value="-8">Forward</button><span></span><button data-axis="roll" data-value="-8">Left</button><button id="center">Level</button><button data-axis="roll" data-value="8">Right</button><button data-axis="yaw" data-value="-25">Yaw left</button><button data-axis="pitch" data-value="8">Back</button><button data-axis="yaw" data-value="25">Yaw right</button></div>
<p><a href="/report" target="_blank" id="report">Last stopped test report</a></p>
<p class="small">If communication is lost for 0.5 s the motors stop. This cuts thrust; it does not land the drone. The previous USB 5 s / 15 s test timers do not apply to live phone control.</p>
<script>
const $=id=>document.getElementById(id);let state=null,want=false,seenArmed=false,armTime=0,inflight=false,seq=0,nonce=0;
const sid=(crypto.getRandomValues(new Uint32Array(1))[0]>>>1)||1;
const cmd={throttle:1000,roll:0,pitch:0,yaw:0};
function body(action){return `${nonce} ${sid} ${++seq} ${action} ${cmd.throttle} ${cmd.roll} ${cmd.pitch} ${cmd.yaw}`}
function resetControls(){cmd.throttle=1000;cmd.roll=cmd.pitch=cmd.yaw=0;$('throttle').value=1000;$('throttleText').textContent=1000;$('throttle').disabled=true;}
function stop(){want=seenArmed=false;resetControls();navigator.sendBeacon('/control',new Blob([body(0)],{type:'text/plain'}));}
async function send(action){if(inflight&&action!==0)return;inflight=true;const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),250);try{const r=await fetch('/control',{method:'POST',body:body(action),signal:abort.signal,cache:'no-store'});if(!r.ok)throw Error('command failed');}catch(e){if(want)stop();}finally{clearTimeout(timer);inflight=false;}}
$('stop').onclick=stop;
$('stop').onpointerdown=e=>{e.preventDefault();stop();};
$('arm').onclick=()=>{if(!state||!state.ready||state.fault||!$('enable').checked)return;resetControls();nonce=state.nonce;want=true;seenArmed=false;armTime=performance.now();send(1);};
$('enable').onchange=()=>{if(!$('enable').checked)stop();};
$('throttle').oninput=e=>{cmd.throttle=Number(e.target.value);$('throttleText').textContent=cmd.throttle;};
$('center').onclick=()=>{cmd.roll=cmd.pitch=cmd.yaw=0;};
for(const b of document.querySelectorAll('[data-axis]')){b.onpointerdown=e=>{e.preventDefault();if(!seenArmed)return;b.setPointerCapture(e.pointerId);cmd[b.dataset.axis]=Number(b.dataset.value);};const release=()=>{cmd[b.dataset.axis]=0;};b.onpointerup=release;b.onpointercancel=release;b.onlostpointercapture=release;}
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});window.addEventListener('pagehide',stop);window.addEventListener('blur',()=>{if(want)stop();});
$('report').onclick=()=>{stop();};
setInterval(()=>{if(want)send(2);},100);
async function poll(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error('status failed');state=await r.json();if(want&&state.nonce!==nonce){want=seenArmed=false;resetControls();}if(want&&state.armed){seenArmed=true;$('throttle').disabled=false;}else if(want&&((seenArmed&&!state.armed)||performance.now()-armTime>1500)){want=seenArmed=false;resetControls();}
$('arm').disabled=want||!state.canArm||!$('enable').checked;$('status').textContent=`${state.armed?'ARMED':'DISARMED'} | ${state.ready?'READY':'CALIBRATING '+state.calibration+'/600'}\nRoll ${state.roll.toFixed(1)}°  Pitch ${state.pitch.toFixed(1)}°\nBase ${state.base} / ${state.target} us\nFL7 FR5 RR3 RL2: ${state.motors.join(' ')}\n${state.reason}`+(state.ready&&!state.canArm&&!state.armed&&!state.fault?'\nArming waits for the 5-second hold and a level, still frame.':'');
}catch(e){$('status').textContent='Connection lost — reconnect, then arm manually.';$('arm').disabled=true;if(want)stop();}finally{setTimeout(poll,250);}}poll();
</script></html>
)PAGE";
