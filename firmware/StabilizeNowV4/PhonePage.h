#pragma once
static const char PHONE_PAGE[] PROGMEM=R"PAGE(
<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Quad Remote Control</title><style>
body{font:17px system-ui;background:#111a24;color:#edf3fa;max-width:540px;margin:20px auto;padding:0 16px}h1{font-size:25px}button{font:inherit;border:0;border-radius:10px;padding:16px;background:#31465b;color:white;touch-action:none}button:disabled{opacity:.4}.stop{background:#c32335;width:100%;font-weight:800;font-size:24px;margin:14px 0}.arm{background:#187653;width:100%}input[type=range]{width:100%;height:44px}pre{white-space:pre-wrap;background:#1e2d3d;padding:12px;border-radius:8px;font-size:14px}.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:16px 0}.small{font-size:14px;color:#b8c8d8}a{color:#a7d4ff}
input[type=number]{width:78px;max-width:100%;font:14px system-ui;padding:4px}select{max-width:100%;font:14px system-ui}td{padding:3px}
</style><h1>Quad Remote Control</h1>
<p class="small">Experimental self-level controller. MPU X arrow forward, Y arrow left. Calibrate level and still. First checks: remove all props. No automatic takeoff, height or position hold.</p>
<pre id="status">Connecting...</pre>
<button id="tuneReport">Show autotune results</button>
<pre id="tuneOutput" hidden>Autotune results appear here.</pre>
<p><button id="settingsToggle">Tuning and Wi-Fi tests</button></p>
<section id="settingsBody" hidden>
<p class="small">Change tuning while motors are stopped. Manual settings are unverified. Apply uses RAM; Save applied values survives power-off. Test starts require READY, a level/still frame and motor-output enable below.</p>
<table style="width:100%"><tr><th>Axis</th><th>P</th><th>I</th><th>D</th></tr>
<tr><td>Roll</td><td><input id="g0" type="number" min="0.25" max="1.2" step="0.01"></td><td><input id="g1" type="number" min="0.02" max="0.3" step="0.01"></td><td><input id="g2" type="number" min="0.0005" max="0.006" step="0.0001"></td></tr>
<tr><td>Pitch</td><td><input id="g3" type="number" min="0.25" max="1.2" step="0.01"></td><td><input id="g4" type="number" min="0.02" max="0.3" step="0.01"></td><td><input id="g5" type="number" min="0.0005" max="0.006" step="0.0001"></td></tr>
<tr><td>Yaw</td><td><input id="g6" type="number" min="0.1" max="2" step="0.01"></td><td><input id="g7" type="number" min="0" max="0.5" step="0.01"></td><td><input id="g8" type="number" min="0" max="0.006" step="0.0001"></td></tr></table>
<p class="small">Integral limits (us): roll <input id="g9" type="number" min="25" max="60" step="1"> pitch <input id="g10" type="number" min="25" max="60" step="1"> yaw <input id="g11" type="number" min="0" max="30" step="1">.<br>Level strength <input id="g12" type="number" min="1" max="5" step="0.1"> (angle-to-rate gain).</p>
<p><b>Motor balance trim</b></p>
<p class="small">Compensate a steady motor-power imbalance while still requesting level. Values are microseconds per motor at base 1400, fading smoothly from zero at 1100. Existing output limits remain active. This does not measure the centre of mass automatically.</p>
<p><label>Left/right trim <input id="g13" type="number" min="-40" max="40" step="1" value="0"></label><br><span class="small">Positive boosts LEFT; negative boosts RIGHT.</span></p>
<p><label>Front/rear trim <input id="g14" type="number" min="-40" max="40" step="1" value="0"></label><br><span class="small">Positive boosts FRONT; negative boosts REAR. Boosting a pair reduces the opposing pair equally.</span></p>
<p class="small">Starts at zero. Adjust only while stopped and apply temporarily before saving. The equal-pulse motor check ignores trim. Balance trim stays active in stabilized and response tests.</p>
<p><button id="applySettings">Apply temporarily</button> <button id="saveSettings">Save applied values</button> <button id="reloadSettings">Reload active values</button></p>
<p><button id="calibrate">Calibrate level</button> <button id="factorySettings">Factory defaults</button></p>
<pre id="settingsInfo">Open this panel to load active values.</pre>
<p class="small">Bounded tests require a secured fixture; no free-flight testing. Response checks need the tested axis free to rotate. Equal-pulse check is for bare motors with stabilization disabled. STOP and connection-loss shutdown stay active.</p>
<label>Test <select id="testKind"><option value="5">Stabilized level test</option><option value="6">Equal 1250 us, props off</option><option value="7">Roll response check</option><option value="8">Pitch response check</option><option value="9">Lower pitch-P hold check</option></select></label>
<p><label>Power (us) <input id="testPulse" type="number" min="1100" max="1400" step="10" value="1330"></label> <label>Seconds <select id="testSeconds"><option value="3">3</option><option value="5">5</option><option value="7">7</option></select></label></p>
<button id="runWifiTest">Start one bounded test</button>
<p class="small">Keep this page active. Test power is fixed for the run; keyboard/slider steering is disabled during tests. Every run stops automatically; none repeat.</p>
<button id="readReport">Show stopped test report</button><pre id="webReport" hidden></pre>
</section>
<label><input id="enable" type="checkbox"> Enable physical motor outputs</label>
<p><label><input id="keyboard" type="checkbox" checked> Keyboard controls</label></p>
<p class="small"><b>1.</b> Tick Enable physical motor outputs. <b>2.</b> Click ARM once. <b>3.</b> Hold W gently to increase power; S decreases it. No Shift key needed.</p><table style="width:100%;text-align:left"><tr><th>Key</th><th>What it does</th></tr><tr><td>W / S</td><td>More / less motor power (not automatic height control)</td></tr><tr><td>Arrow keys</td><td>Tilt forward / backward / left / right</td></tr><tr><td>A / D</td><td>Turn the whole drone left / right</td></tr><tr><td>Space / Esc</td><td>STOP all motors immediately</td></tr></table><p class="small">Release steering keys to request level; throttle stays where you left it. Switching windows or losing connection stops the motors.</p>
<button id="arm" class="arm" disabled>ARM at zero throttle</button>
<p id="armHint" class="small" role="status">ARM disabled: waiting for controller status.</p>
<button id="stop" class="stop">STOP MOTORS</button>
<label>Throttle: <b id="throttleText">1000</b> &micro;s</label><input id="throttle" type="range" min="1000" max="1700" step="5" value="1000" disabled>
<p class="small">The controller varies all four motors to follow the requested tilt. Release steering to request level. Keep this page active. On-screen steering works with keyboard mode off.</p>
<div class="pad"><span></span><button data-axis="pitch" data-value="-8">Forward</button><span></span><button data-axis="roll" data-value="-8">Left</button><button id="center">Level</button><button data-axis="roll" data-value="8">Right</button><button data-axis="yaw" data-value="-25">Yaw left</button><button data-axis="pitch" data-value="8">Back</button><button data-axis="yaw" data-value="25">Yaw right</button></div>
<p><a href="/report" target="_blank" id="report">Last stopped test report</a></p>
<p class="small">If communication is lost for 0.5 s the motors stop. This cuts thrust; it does not land the drone. The USB seven-second trial limit does not apply to live remote control.</p>
<script>
const $=id=>document.getElementById(id);let state=null,want=false,seenArmed=false,armTime=0,inflight=false,seq=0,nonce=0,remoteRun=false,settingsBusy=false;
const sid=(crypto.getRandomValues(new Uint32Array(1))[0]>>>1)||1;
const cmd={throttle:1000,roll:0,pitch:0,yaw:0};
const keys=new Set(),motionKeys=new Set(['KeyW','KeyS','KeyA','KeyD','ArrowUp','ArrowDown','ArrowLeft','ArrowRight']);
const keyboardMode=()=>$('keyboard').checked;
function armingHint(){if(!state)return 'ARM disabled: waiting for controller status.';if(want)return seenArmed?'ARMED. W/S changes power; arrows tilt; A/D turns. Space/Esc stops.':'Waiting for ARM acknowledgement; throttle stays zero.';if(readingTune)return 'ARM disabled: report is loading. Wait a moment.';if(state.fault)return 'ARM disabled: controller fault. See status above.';if(!state.ready)return 'ARM disabled: calibrating. Keep the frame level and still.';if(state.armed)return 'ARM disabled: another controller or USB test is operating the motors.';if(state.tuning)return 'ARM disabled: USB test is still active.';if(Math.abs(state.roll)>5||Math.abs(state.pitch)>5)return `ARM disabled: roll ${state.roll.toFixed(1)}° / pitch ${state.pitch.toFixed(1)}°. Both must be within ±5°. Set the frame level.`;if(!$('enable').checked)return 'ARM disabled: tick Enable physical motor outputs.';if(!state.canArm)return 'ARM disabled by controller: keep frame still and wait for startup.';return 'Ready: click ARM once at zero throttle. No key needs to be held.';}
let tuneVisible=false,readingTune=false,lastTuneRead=-2000,shownTuneRev=-1;
async function loadTune(){if(readingTune||want||!tuneVisible)return;readingTune=true;lastTuneRead=performance.now();const revision=Number(state?.tuneRev||0),abort=new AbortController(),timer=setTimeout(()=>abort.abort(),750);try{const r=await fetch('/tune',{cache:'no-store',signal:abort.signal});if(!r.ok)throw Error('report failed');$('tuneOutput').textContent=await r.text();shownTuneRev=revision;}catch(e){$('tuneOutput').textContent='Autotune report unavailable. Reconnect, then refresh this view.';}finally{clearTimeout(timer);readingTune=false;}}
$('tuneReport').onclick=()=>{if(want)return;tuneVisible=!tuneVisible;$('tuneOutput').hidden=!tuneVisible;$('tuneReport').textContent=tuneVisible?'Hide autotune results':'Show autotune results';if(tuneVisible){shownTuneRev=-1;loadTune();}};
function body(action){return `${nonce} ${sid} ${++seq} ${action} ${cmd.throttle} ${cmd.roll} ${cmd.pitch} ${cmd.yaw}`}
function resetControls(){remoteRun=false;keys.clear();cmd.throttle=1000;cmd.roll=cmd.pitch=cmd.yaw=0;$('throttle').value=1000;$('throttleText').textContent=1000;$('throttle').disabled=true;}
function stop(){want=seenArmed=false;resetControls();navigator.sendBeacon('/control',new Blob([body(0)],{type:'text/plain'}));}
async function send(action){if(inflight&&action!==0)return;inflight=true;const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),250);try{const r=await fetch('/control',{method:'POST',body:body(action),signal:abort.signal,cache:'no-store'});if(!r.ok)throw Error('command failed');}catch(e){if(want)stop();}finally{clearTimeout(timer);inflight=false;}}
$('stop').onclick=stop;
$('stop').onpointerdown=e=>{e.preventDefault();stop();};
$('arm').onclick=()=>{if(settingsBusy||readingTune||!state||!state.canArm||!state.ready||state.fault||!$('enable').checked)return;resetControls();nonce=state.nonce;want=true;seenArmed=false;armTime=performance.now();send(1);};
$('enable').onchange=()=>{if(!$('enable').checked)stop();};
$('keyboard').onchange=()=>{if(want||seenArmed)stop();else resetControls();};
$('throttle').oninput=e=>{cmd.throttle=Number(e.target.value);$('throttleText').textContent=cmd.throttle;};
$('center').onclick=()=>{for(const key of motionKeys)keys.delete(key);cmd.roll=cmd.pitch=cmd.yaw=0;};
for(const b of document.querySelectorAll('[data-axis]')){b.onpointerdown=e=>{e.preventDefault();if(remoteRun||!seenArmed||keyboardMode())return;b.setPointerCapture(e.pointerId);cmd[b.dataset.axis]=Number(b.dataset.value);};const release=()=>{if(!keyboardMode())cmd[b.dataset.axis]=0;};b.onpointerup=release;b.onpointercancel=release;b.onlostpointercapture=release;}
function keyboardSteering(){cmd.roll=4*(Number(keys.has('ArrowRight'))-Number(keys.has('ArrowLeft')));cmd.pitch=4*(Number(keys.has('ArrowDown'))-Number(keys.has('ArrowUp')));cmd.yaw=20*(Number(keys.has('KeyD'))-Number(keys.has('KeyA')));}
const editing=e=>['INPUT','SELECT','TEXTAREA'].includes(e.target?.tagName);
document.addEventListener('keydown',e=>{if(e.code==='Space'||e.code==='Escape'){e.preventDefault();stopIfControlling();return;}if(remoteRun||editing(e)||!keyboardMode())return;if(!motionKeys.has(e.code))return;e.preventDefault();if(!want||!seenArmed)return;keys.add(e.code);keyboardSteering();});
document.addEventListener('keyup',e=>{keys.delete(e.code);if(remoteRun||editing(e)||!keyboardMode())return;if(motionKeys.has(e.code)){e.preventDefault();keyboardSteering();}});
function stopIfControlling(){keys.clear();if(want||seenArmed)stop();}
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopIfControlling();});window.addEventListener('pagehide',stopIfControlling);window.addEventListener('blur',stopIfControlling);
$('report').onclick=()=>{stop();};
setInterval(()=>{if(!want)return;if(!remoteRun&&keyboardMode()){if(seenArmed){keyboardSteering();cmd.throttle=Math.max(1000,Math.min(1700,cmd.throttle+10*(Number(keys.has('KeyW'))-Number(keys.has('KeyS')))));$('throttle').value=cmd.throttle;$('throttleText').textContent=cmd.throttle;}}send(2);},100);
async function poll(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error('status failed');state=await r.json();const wasRemote=remoteRun;if(want&&state.nonce!==nonce){want=seenArmed=false;resetControls();}if(want&&state.armed){seenArmed=true;$('throttle').disabled=remoteRun;}else if(want&&((seenArmed&&!state.armed)||performance.now()-armTime>1500)){want=seenArmed=false;resetControls();}if(wasRemote&&!want)loadWebReport();
$('armHint').textContent=armingHint();$('arm').disabled=want||settingsBusy||readingTune||!state.canArm||!$('enable').checked;$('keyboard').disabled=want;$('tuneReport').disabled=want;$('status').textContent=`${state.armed?'ARMED':'DISARMED'} | ${state.ready?'READY':'CALIBRATING '+state.calibration+'/600'}\nRoll ${state.roll.toFixed(1)}°  Pitch ${state.pitch.toFixed(1)}°\nYaw rate ${Number(state.yawRate||0).toFixed(1)}°/s / request ${Number(state.yawTarget||0).toFixed(0)}°/s / trim ${Number(state.yawI||0).toFixed(1)} us\nBase ${state.base} / ${state.target} us\nFL0 FR19 RR18 RL3: ${state.motors.join(' ')}\n${state.reason}`+(keyboardMode()?`\nKEYBOARD: W/S throttle | arrows tilt | A/D yaw | Space STOP`:'')+(state.tuning?`\nAUTOTUNE ${state.trial}/${state.trials} | ${state.armed?(Number(state.trialMs||0)/1000).toFixed(1)+' / 7.0 s':'motors OFF; waiting for next trial'}`:'')+(state.ready&&!state.canArm&&!state.armed&&!state.fault&&!state.tuning?'\nController requires its startup delay plus a level, still frame. See the ARM message below.':'');
if(tuneVisible&&!want&&shownTuneRev!==Number(state.tuneRev||0)&&performance.now()-lastTuneRead>=2000)loadTune();
updateSetupButtons();if(state.wifiTest)$('armHint').textContent=`Wi-Fi test: ${(Number(state.trialMs||0)/1000).toFixed(1)} / ${Number(state.testLimitMs||0)/1000} seconds. STOP cancels.`;
if(!$('settingsBody').hidden&&!want&&!settingsBusy&&!settingsReading&&lastSettingsRev!==Number(state.tuneRev||0))readOnline(false).catch(()=>{});
}catch(e){$('status').textContent='Connection lost — reconnect, then arm manually.';$('armHint').textContent='ARM disabled: connection lost. Reconnect, then arm manually.';$('arm').disabled=true;if(want)stop();}finally{setTimeout(poll,250);}}poll();
const bounds=[[.25,1.2],[.02,.3],[.0005,.006],[.25,1.2],[.02,.3],[.0005,.006],[.1,2],[0,.5],[0,.006],[25,60],[25,60],[0,30],[1,5],[-40,40],[-40,40]];
let settingsReading=false,lastSettingsRev=-1;
function updateSetupButtons(){const busy=settingsBusy||want||!!state?.setupBusy||!!state?.armed;for(const id of ['applySettings','saveSettings','reloadSettings','calibrate','factorySettings'])$(id).disabled=busy;for(let i=0;i<15;i++)$('g'+i).disabled=busy;$('runWifiTest').disabled=busy||!state?.canArm||!$('enable').checked;}
async function readOnline(fill){settingsReading=true;const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),750);try{const r=await fetch('/settings',{cache:'no-store',signal:abort.signal});if(!r.ok)throw Error('settings unavailable');const s=await r.json();if(!Array.isArray(s.values)||s.values.length!==15||!s.values.every(Number.isFinite))throw Error('Settings version mismatch: reload this page after updating firmware.');if(fill)s.values.forEach((v,i)=>$('g'+i).value=v);lastSettingsRev=Number(state?.tuneRev||0);$('settingsInfo').textContent=`${s.result}\nStorage: ${s.storage}\nManual tuning is not verified for flight.`;return s;}finally{clearTimeout(timer);settingsReading=false;}}
$('settingsToggle').onclick=async()=>{$('settingsBody').hidden=!$('settingsBody').hidden;if(!$('settingsBody').hidden){try{await readOnline(true);}catch(e){$('settingsInfo').textContent=e.message;}}};
function inputValues(){return bounds.map(([lo,hi],i)=>{const raw=String($('g'+i).value).trim(),v=Number(raw);if(!raw||!Number.isFinite(v)||v<lo||v>hi)throw Error(`Value ${i+1} must be ${lo}..${hi}`);return v;});}
async function setupCommand(kind,values){
 if(settingsBusy||want||!state||state.armed||state.setupBusy)return;
 const start=kind>=5;if(start&&(!state.canArm||!state.ready||state.fault||!$('enable').checked))return;
 let pulse=0,seconds=0;
 if(start){pulse=kind===6?1250:Number($('testPulse').value);seconds=kind===6?3:(kind===5?Number($('testSeconds').value):7);if(!Number.isInteger(pulse)||pulse<(kind>=7?1250:1100)||pulse>1400||![3,5,7].includes(seconds)){$('settingsInfo').textContent='Test power/duration outside allowed range.';return;}}
 settingsBusy=true;updateSetupButtons();const requestSeq=++seq,requestNonce=state.nonce;
 if(start){resetControls();nonce=requestNonce;want=remoteRun=true;seenArmed=false;armTime=performance.now();}
 const text=`${requestNonce} ${sid} ${requestSeq} ${kind} ${pulse} ${seconds}`+(kind===1?' '+values.join(' '):'');
 const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),750);
 try{const r=await fetch('/setup',{method:'POST',body:text,cache:'no-store',signal:abort.signal});if(!r.ok)throw Error('Settings/test request rejected');let ack=false;
  for(let i=0;i<10;i++){const s=await readOnline(false);if(s.ackSession===sid&&s.ackSequence===requestSeq){ack=true;break;}await new Promise(resolve=>setTimeout(resolve,100));}
  if(!ack)throw Error('No acknowledgement. Check active values; request will not retry.');await poll();
 }catch(e){$('settingsInfo').textContent=e.message;if(start&&want)stop();}finally{clearTimeout(timer);settingsBusy=false;updateSetupButtons();}
}
$('applySettings').onclick=()=>{try{return setupCommand(1,inputValues());}catch(e){$('settingsInfo').textContent=e.message;}};
$('saveSettings').onclick=()=>setupCommand(2);
$('calibrate').onclick=()=>setupCommand(3);
$('factorySettings').onclick=()=>setupCommand(4);
$('runWifiTest').onclick=()=>setupCommand(Number($('testKind').value));
$('reloadSettings').onclick=async()=>{try{await readOnline(true);}catch(e){$('settingsInfo').textContent=e.message;}};
async function loadWebReport(){try{const r=await fetch('/report',{cache:'no-store'});if(!r.ok)throw Error('report unavailable');$('webReport').textContent=await r.text();$('webReport').hidden=false;}catch(e){$('webReport').textContent=e.message;$('webReport').hidden=false;}}
$('readReport').onclick=()=>{if(!want)loadWebReport();};
</script></html>
)PAGE";
