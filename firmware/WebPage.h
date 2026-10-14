#pragma once

// Tuning page served at http://192.168.4.1 (robot's own WiFi "DengeAraci").
// Parameters are listed from /params, live values are polled from /state, and every
// button just sends a text command to /cmd — the same commands as serial and UDP.

static const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Denge Aracı</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;background:#101418;color:#e6e6e6}
header{padding:10px 12px;background:#1b222a;position:sticky;top:0;z-index:2}
#st{font-size:13px;color:#9fb3c8;margin-top:4px;word-break:break-all}
.state{font-weight:700;padding:2px 8px;border-radius:6px;background:#334}
main{padding:10px 12px;max-width:900px;margin:auto}
canvas{width:100%;height:240px;background:#0b0e11;border-radius:8px}
.btns{display:flex;flex-wrap:wrap;gap:6px;margin:10px 0}
button{background:#2b6cb0;color:#fff;border:0;border-radius:6px;padding:9px 12px;font-size:14px}
button.red{background:#c53030}button.gray{background:#4a5568}
table{width:100%;border-collapse:collapse;font-size:14px}
td{padding:5px 4px;border-bottom:1px solid #222a33;vertical-align:middle}
td.n{font-family:monospace;white-space:nowrap}td.d{color:#8896a5;font-size:12px}
input{background:#0b0e11;color:#fff;border:1px solid #3a4655;border-radius:5px;padding:6px;width:90px}
input.wide{width:160px}
#msg{font-family:monospace;font-size:13px;white-space:pre-wrap;color:#9ae6b4;min-height:18px}
.legend span{margin-right:10px;font-size:12px}
details{margin:12px 0}
</style></head><body>
<header><div><b>Denge Aracı</b> <span id="state" class="state">...</span></div><div id="st"></div></header>
<main>
<canvas id="cv" width="900" height="240"></canvas>
<div class="legend"><span style="color:#63b3ed">■ eğim</span><span style="color:#f6ad55">■ hedef eğim</span><span style="color:#68d391">■ motor %</span><span style="color:#d6bcfa">■ konum</span></div>
<div class="btns">
<button class="red" onclick="c('stop')">Durdur</button>
<button onclick="c('start')">Başlat</button>
<button onclick="c('zero')">Burası merkez</button>
<button onclick="c('save')">Kaydet</button>
<button class="gray" onclick="c('calib')">Gyro kalibre</button>
<button class="gray" onclick="c('kick 25 80')">Test itmesi</button>
<button class="gray" onclick="loadParams()">Yenile</button>
<button class="gray" onclick="if(confirm('Varsayılanlar yüklensin mi?'))c('defaults').then(loadParams)">Varsayılan</button>
</div>
<div id="msg"></div>
<table id="pt"></table>
<details><summary>Komut satırı</summary>
<input id="cmd" class="wide" placeholder="ör: kp=7 kd=0.4"><button onclick="c(cmd.value)">Gönder</button></details>
<details><summary>İstasyon WiFi (PC hotspot vb.)</summary>
<input id="ss" class="wide" placeholder="SSID"> <input id="pw" class="wide" type="password" placeholder="Şifre">
<button onclick="c('wifi &quot;'+ss.value+'&quot; &quot;'+pw.value+'&quot;')">Bağlan</button></details>
</main>
<script>
const $=id=>document.getElementById(id);
async function c(cmd){try{const r=await fetch('/cmd?c='+encodeURIComponent(cmd));const t=await r.text();$('msg').textContent=t;return t}catch(e){$('msg').textContent='bağlantı hatası'}}
async function loadParams(){
 const r=await fetch('/params');const list=await r.json();const t=$('pt');t.innerHTML='';
 for(const p of list){const tr=document.createElement('tr');
  tr.innerHTML='<td class="n">'+p.n+'</td><td><input type="number" step="any" value="'+(+p.v)+'"></td><td class="d">'+p.d+' ['+(+p.lo)+' … '+(+p.hi)+']</td>';
  const inp=tr.querySelector('input');inp.onchange=()=>c('set '+p.n+' '+inp.value);t.appendChild(tr);}
}
const N=250,H={a:[],t:[],u:[],x:[]};
function push(k,v){H[k].push(v);if(H[k].length>N)H[k].shift()}
function draw(trim){
 const cv=$('cv'),g=cv.getContext('2d'),w=cv.width,h=cv.height,m=h/2;
 g.clearRect(0,0,w,h);g.strokeStyle='#223';g.beginPath();g.moveTo(0,m/2);g.lineTo(w,m/2);g.moveTo(0,m+m/2);g.lineTo(w,m+m/2);g.moveTo(0,m);g.lineTo(w,m);g.stroke();
 const line=(arr,col,y0,scale)=>{g.strokeStyle=col;g.beginPath();arr.forEach((v,i)=>{const x=i*w/(N-1),y=y0-v*scale;i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke()};
 line(H.a.map(v=>v-trim),'#63b3ed',m/2,(m/2)/15);line(H.t.map(v=>v-trim),'#f6ad55',m/2,(m/2)/15);
 line(H.u,'#68d391',m+m/2,(m/2)/100);line(H.x,'#d6bcfa',m+m/2,(m/2)/50);
 g.fillStyle='#667';g.font='12px sans-serif';g.fillText('eğim-trim ±15°',4,12);g.fillText('motor ±100%  konum ±50',4,m+12);
}
async function poll(){
 try{const s=await(await fetch('/state')).json();
  push('a',s.a);push('t',s.t);push('u',s.u);push('x',s.x);draw(s.tr);
  $('state').textContent=s.s;
  $('st').textContent='eğim '+s.a.toFixed(1)+'°  trim '+s.tr.toFixed(2)+'°  konum '+s.x.toFixed(1)+'  hız '+s.v.toFixed(1)+'  yön '+s.y.toFixed(0)+'°  döngü '+s.hz+' Hz (max '+s.dtmax+' µs)  düşme '+s.falls+'  pil '+(s.vb>0?s.vb.toFixed(2)+' V':'ölçülmüyor')+'  |  '+s.net;
 }catch(e){$('state').textContent='bağlantı yok'}
 setTimeout(poll,60);
}
loadParams();poll();
</script></body></html>)HTML";
