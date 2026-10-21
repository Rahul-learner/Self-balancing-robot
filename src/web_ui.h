#pragma once

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Balance Bot PID Tuner</title>
<style>
:root{--bg:#0f1115;--card:#181b22;--fg:#e8eaf0;--mut:#8b93a7;--ok:#2ecc71;--bad:#e74c3c;--acc:#4da3ff;--warn:#f5a623;--purp:#a56eff}
*{box-sizing:border-box}
body{margin:0;padding:12px;background:var(--bg);color:var(--fg);font:14px system-ui,-apple-system,sans-serif;max-width:760px;margin:auto}
h1{font-size:20px;margin:4px 0 10px;display:flex;justify-content:space-between;align-items:center}
.card{background:var(--card);border-radius:10px;padding:12px;margin-bottom:12px;border:1px solid #232733}
.bar{display:flex;flex-wrap:wrap;gap:8px 18px;align-items:center}
.badge{padding:4px 10px;border-radius:12px;font-weight:700;font-size:12px;background:#333}
.b0{background:#444}.b1{background:var(--warn);color:#000}.b2{background:var(--ok);color:#000}.b3,.b4{background:var(--bad)}
.stat{color:var(--mut)}.stat b{color:var(--fg);font-variant-numeric:tabular-nums}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
button{background:#2a2f3a;color:var(--fg);border:0;border-radius:8px;padding:9px 14px;font:inherit;cursor:pointer;transition:filter .15s}
button:hover{filter:brightness(1.15)}
button:active{filter:brightness(1.3)}
.go{background:var(--ok);color:#000;font-weight:700}.stp{background:var(--bad);font-weight:700}
canvas#plot{width:100%;height:190px;background:#0b0d11;border-radius:8px;display:block}
.leg{font-size:12px;color:var(--mut);margin-top:6px;display:flex;justify-content:space-between;flex-wrap:wrap;gap:6px}
.p{display:grid;grid-template-columns:1fr 90px;gap:4px 10px;align-items:center;margin:10px 0}
.p label{grid-column:1/3;color:var(--mut);font-size:13px;display:flex;justify-content:space-between}
.p label span{color:var(--acc);font-variant-numeric:tabular-nums}
.p input[type=range]{width:100%;accent-color:var(--acc)}
.p input[type=number]{width:100%;background:#0b0d11;color:var(--fg);border:1px solid #2a2f3a;border-radius:6px;padding:6px}
#msg{color:var(--acc);font-size:13px;min-height:18px;margin-top:6px}
.tips{font-size:12.5px;color:var(--mut);line-height:1.5;margin:0;padding-left:18px}
.tips b{color:var(--fg)}
</style></head><body>
<h1><span>Balance Bot &mdash; Upright PID Tuner</span><span id="st" class="badge b0">CONNECTING...</span></h1>

<div class="card">
  <div class="bar">
    <span class="stat">pitch <b id="pitch">0.00</b>&deg;</span>
    <span class="stat">tilt err <b id="tilt">0.00</b>&deg;</span>
    <span class="stat">gyro rate <b id="rate">0.0</b>&deg;/s</span>
    <span class="stat">pwm L/R <b id="pwm">0 / 0</b></span>
    <span class="stat">enc L/R <b id="enc">0 / 0</b></span>
    <span class="stat">spd <b id="spd">0</b> t/s</span>
    <span class="stat">loop <b id="hz">0</b> Hz</span>
  </div>
</div>

<div class="card">
  <div class="row">
    <button class="go" id="arm">ARM</button>
    <button class="go" id="force" style="background:#29b6f6;color:#000">FORCE START</button>
    <button class="stp" id="stop">STOP (SPACE)</button>
    <button id="zero">Set Balance Angle = Current Pitch</button>
    <button id="recal">Recalibrate Gyro</button>
  </div>
  <div id="msg"></div>
  <div class="leg"><span>Hold near balance angle for 0.15s to arm, or click <b>FORCE START</b> to engage motors immediately.</span></div>
</div>

<div class="card">
  <canvas id="plot" width="740" height="190"></canvas>
  <div class="leg">
    <div>
      <span style="color:#4da3ff">&#9632;</span> tilt error (&plusmn;40&deg;) &nbsp;
      <span style="color:#a56eff">&#9632;</span> gyro rate (&plusmn;200&deg;/s) &nbsp;
      <span style="color:#f5a623">&#9632;</span> motor PWM (&plusmn;255)
    </div>
    <button id="pausePlot" style="padding:3px 10px;font-size:12px">Pause Plot</button>
  </div>
</div>

<div class="card">
  <div id="params"></div>
  <div class="row" style="margin-top:10px">
    <button id="save" class="go">Save to Flash</button>
    <button id="defaults">Reset to Defaults</button>
  </div>
  <div class="leg" style="margin-top:6px">
    <span><b>Save is deferred while RUNNING</b> to protect the balance loop &mdash; it commits automatically once you disarm.</span>
  </div>
</div>

<script>
const $=id=>document.getElementById(id);
const STATES=["DISARMED","WAITING (hold near balance)","RUNNING","FALLEN — stand up near balance, or press STOP","IMU ERROR"];
const DEFAULT_START_WINDOW=8.0;

// FIX #3: cache current param values so the READY indicator tracks the live
// startWindow setting instead of a hardcoded 8.0.
const paramValues={};

function msg(t){$("msg").textContent=t;clearTimeout(msg.t);msg.t=setTimeout(()=>$("msg").textContent="",2500);}

// FIX #2: surface non-2xx responses to the user (e.g. 409 "disarm first"
// from /recal, 400 "bad value" from /set). Returns null on error so callers
// can distinguish success from failure without inspecting HTTP codes.
function call(url,m){
  return fetch(url).then(r=>r.text().then(body=>{
    if(!r.ok){ msg("Error: "+body); return null; }
    if(m) msg(m);
    return body;
  })).catch(()=>{msg("No connection to robot"); return null;});
}

function send(n,v){
  paramValues[n]=parseFloat(v);
  call("/set?name="+n+"&value="+v);
}

function build(list){
  const box=$("params");box.innerHTML="";
  list.forEach(p=>{
    paramValues[p.n]=parseFloat(p.v);
    const d=document.createElement("div");d.className="p";
    if(p.n==="motorDir"){
      // FIX #4: <span> instead of the non-standard <sp> element
      d.innerHTML='<label style="cursor:pointer"><span><input type="checkbox" id="p_'+p.n+'"> Reverse motor direction (tick if robot drives away from a fall)</span></label>';
      box.appendChild(d);
      const c=d.querySelector("input");c.checked=p.v<0;
      c.onchange=()=>send(p.n,c.checked?-1:1);
      return;
    }
    d.innerHTML='<label><b>'+p.l+'</b><span id="v_'+p.n+'">'+p.v+'</span></label>'+
                '<input type="range" min="'+p.min+'" max="'+p.max+'" step="'+p.s+'" value="'+p.v+'">'+
                '<input type="number" min="'+p.min+'" max="'+p.max+'" step="'+p.s+'" value="'+p.v+'">';
    const lbl=d.querySelector("span"),r=d.children[1],n=d.children[2];
    let dbTimer=null;
    r.oninput=()=>{n.value=r.value;lbl.textContent=r.value;clearTimeout(dbTimer);dbTimer=setTimeout(()=>send(p.n,r.value),80);};
    r.onchange=()=>{clearTimeout(dbTimer);send(p.n,r.value);};
    n.onchange=()=>{r.value=n.value;lbl.textContent=n.value;send(p.n,n.value);};
    box.appendChild(d);
  });
}

function loadParams(){
  fetch("/params").then(r=>r.json()).then(build)
    .catch(()=>msg("Could not load parameters"));
}

$("arm").onclick=()=>call("/arm?on=1","Armed: hold near balance angle");
$("force").onclick=()=>call("/arm?on=1&force=1","Force Started: Motors Active!");
$("stop").onclick=()=>call("/arm?on=0","Stopped");
$("zero").onclick=()=>call("/zero","Balance angle updated — will save when disarmed").then(loadParams);
$("recal").onclick=()=>call("/recal","Recalibrating gyro: keep still (disarm first)");
$("save").onclick=()=>call("/save").then(r=>{
  if(r===null) return;   // error already shown by call()
  msg(r==="pending"?"Save queued — will commit when disarmed":"Saved parameters to flash");
});
$("defaults").onclick=()=>call("/defaults","Defaults restored (click Save to keep)").then(loadParams);

window.addEventListener("keydown",e=>{
  if(e.code==="Space"&&document.activeElement.tagName!=="INPUT"){
    e.preventDefault();call("/arm?on=0","Emergency Stop");
  }
});

const N=300,tlt=[],rt=[],out=[];
let paused=false;
$("pausePlot").onclick=()=>{paused=!paused;$("pausePlot").textContent=paused?"Resume Plot":"Pause Plot";};
const cv=$("plot"),cx=cv.getContext("2d");

function draw(){
  const W=cv.width,H=cv.height;cx.clearRect(0,0,W,H);
  cx.strokeStyle="#232733";cx.beginPath();cx.moveTo(0,H/2);cx.lineTo(W,H/2);cx.stroke();
  function line(a,sc,col){cx.strokeStyle=col;cx.lineWidth=1.6;cx.beginPath();
    a.forEach((v,i)=>{const x=i/(N-1)*W,y=H/2-Math.max(-1,Math.min(1,v/sc))*(H/2-6);i?cx.lineTo(x,y):cx.moveTo(x,y);});cx.stroke();}
  line(tlt,40,"#4da3ff");line(rt,200,"#a56eff");line(out,255,"#f5a623");
}

let busy=false;
function poll(){
  if(busy)return;busy=true;
  fetch("/telemetry").then(r=>r.json()).then(t=>{
    const s=$("st");
    if(t.s===1){
      // FIX #3: use live value from param cache, fall back to default
      const win=paramValues.startWindow||DEFAULT_START_WINDOW;
      if(Math.abs(t.t)<=win){
        s.textContent="READY (Release robot!)";s.className="badge b2";
      } else {
        s.textContent="WAITING (Tilt to balance angle)";s.className="badge b1";
      }
    } else {
      s.textContent=STATES[t.s]||"?";s.className="badge b"+t.s;
    }
    $("pitch").textContent=t.p.toFixed(2);$("tilt").textContent=t.t.toFixed(2);
    $("rate").textContent=t.r.toFixed(1);$("pwm").textContent=t.pl+" / "+t.pr;
    $("enc").textContent=t.el+" / "+t.er;$("spd").textContent=t.sp.toFixed(0);
    $("hz").textContent=t.hz;
    if(!paused){
      tlt.push(t.t);rt.push(t.r);out.push((t.pl+t.pr)*0.5);
      if(tlt.length>N){tlt.shift();rt.shift();out.shift();}
      draw();
    }
  }).catch(()=>{$("st").textContent="NO LINK";$("st").className="badge b3";})
    .finally(()=>{busy=false;});
}
loadParams();setInterval(poll,100);
</script></body></html>
)rawliteral";