#include "web_task.h"
#include "config.h"
#if VARIO_USE_WIFI
#include "shared_state.h"
#include "tunables.h"
#include "debug_log.h"
#include "boot_sync.h"
#include "vario_task.h"
#include "trend_buffer.h"
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>
#include <math.h>

// ---------------------------------------------------------------------
// NOTE: this AP draws WiFi TX current bursts that can sag a poorly
// decoupled MS5607 supply rail and skew its readings (we've seen this
// exact failure mode before on this project). If pressure gets noisy or
// biased specifically while this page is open, check the barometer's
// power decoupling before suspecting the driver code.
// ---------------------------------------------------------------------

namespace {
    const char* AP_SSID = "Vario-FreeRTOS";
    const char* AP_PASS = "12345678"; // WPA2 minimum length

    WebServer server(80);

    const char PAGE[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Vario FreeRTOS</title>
<style>
body{background:#111;color:#ddd;font-family:monospace;padding:10px}
input{width:80px;font-family:monospace}
button{padding:6px 10px;margin-left:4px}
fieldset{border:1px solid #444;margin:10px 0;padding:8px}
legend{color:#8f8;padding:0 4px}
#tele{background:#000;color:#0ff;padding:8px;white-space:pre-wrap}
#log{height:32vh;overflow:auto;background:#000;color:#0f0;padding:8px;white-space:pre-wrap;word-break:break-all}
.row{margin:4px 0}
.note{color:#aaa;font-size:12px}
.ok{color:#8f8}
.err{color:#f88}
#trendWrap{background:#000;border:1px solid #333;padding:6px;overflow:hidden}
#trendCanvas{width:100%;height:320px;display:block}
</style></head><body>
<h3>Vario FreeRTOS &mdash; canli telemetri</h3>
<pre id='tele'>yukleniyor...</pre>

<fieldset><legend>Irtifa kalibrasyonu</legend>
<div class='row'><label for='knownAltitudeMeters'>Bilinen rakim (m):</label> <input id='knownAltitudeMeters' name='knownAltitudeMeters' type='text' inputmode='decimal' autocomplete='off' placeholder='orn. 975' style='width:130px'></div>
<button onclick='calibrateAltitude()'>Bilinen Rakimla Kalibre Et</button>
<div class='note'>Bu islem bilinen konumdaki rakimi ayarlar; acilista gerekli degildir. Basincin oturmasini bekle. QNH otomatik hesaplanir ve kaydedilir.</div>
<div id='calResult' class='note'></div>
</fieldset>

<fieldset><legend>IMU yer kalibrasyonu</legend>
<div class='note'>Acilista sabit tutmak gerekmez. IMU destegi icin yerde, cihaz hareketsizken bir kez kalibre et. Sonuc sonraki acilislarda kullanilir. Bu islemi ucusta baslatma.</div>
<button id='imuCalButton' onclick='calibrateImu()'>Yerde IMU Kalibre Et (3 sn)</button>
<div id='imuCalResult' class='note'>Durum bekleniyor...</div>
</fieldset>

<fieldset><legend>Trend - son 30 dakika</legend>
<div id='trendWrap'><canvas id='trendCanvas' width='900' height='320'></canvas></div>
<div id='trendInfo' class='note'>trend yukleniyor...</div>
</fieldset>

<fieldset><legend>Kalman / Fuzyon</legend>
<div class='row'>Alpha (tamamlayici filtre 0-1): <input id='alpha' type='number' step='0.001'></div>
<div class='row'>KF_ACCEL_VAR (buyut=baroya guven): <input id='kfA' type='number' step='0.1'></div>
<div class='row'>KF_ACCEL_BIAS_VAR: <input id='kfAB' type='number' step='0.0001'></div>
<div class='row'>KF_BARO_VAR (kucult=baroya guven): <input id='kfBaro' type='number' step='0.01'></div>
<div class='row'>QNH (hPa): <input id='qnh' type='number' step='0.1'></div>
<button onclick='applyFusion()'>Uygula</button>
</fieldset>

<fieldset><legend>Ses</legend>
<div class='row'>Climb deadband (m/s): <input id='deadband' type='number' step='0.01'></div>
<div class='row'>Climb max (m/s): <input id='climbMax' type='number' step='0.1'></div>
<div class='row'>Sink alarm (m/s, negatif): <input id='sinkAlarm' type='number' step='0.1'></div>
<div class='row'>Ton min (Hz): <input id='toneMin' type='number' step='10'></div>
<div class='row'>Ton max (Hz): <input id='toneMax' type='number' step='10'></div>
<div class='row'>Sink tonu (Hz): <input id='sinkTone' type='number' step='10'></div>
<button onclick='applyAudio()'>Uygula</button>
</fieldset>

<h4>Canli log</h4>
<pre id='log'>yukleniyor...</pre>

<script>
// Clear browser-restored form state only when opening/restoring the page.
window.addEventListener('pageshow', () => {
  document.getElementById('knownAltitudeMeters').value = '';
});
async function calibrateImu(){
  const button = document.getElementById('imuCalButton');
  button.disabled = true;
  try {
    const r = await fetch('/calibrateImu', {method:'POST'});
    const d = await r.json();
    if (!r.ok) throw new Error(d.error || ('HTTP ' + r.status));
    document.getElementById('imuCalResult').textContent = 'Kalibrasyon istendi; cihazi 3 saniye hareketsiz tut.';
  } catch(e) {
    alert('IMU kalibrasyonu: ' + e.message);
    button.disabled = false;
  }
}
async function refreshTele(){
  try {
    const r = await fetch('/status');

    if (!r.ok) {
      throw new Error("HTTP " + r.status);
    }

    const d = await r.json();

    const calMessages = [
      d.imuCalibrated ? 'Kayitli kalibrasyon kullanilabilir.' : 'Kayit yok; barometre ile calisiyor.',
      'Kalibrasyon suruyor; cihazi yerde hareketsiz tut.',
      'Kalibrasyon kaydedildi; sonraki acilislarda tekrar gerekmez.',
      'Kalibrasyon reddedildi: hareket veya yetersiz olcum. Onceki kayit korundu.',
      'Kalibrasyon kaydedilemedi. Onceki kayit korundu.'
    ];
    document.getElementById('imuCalResult').textContent = calMessages[d.imuCalibrationStatus] || calMessages[0];
    document.getElementById('imuCalButton').disabled = !d.imuOk || d.imuCalibrationStatus === 1;
    document.getElementById('tele').textContent =
      `ALTITUDE       = ${d.alt.toFixed(2)} m\n` +
      `BARO ALTITUDE  = ${d.baroAlt.toFixed(2)} m\n` +
      `PRESSURE       = ${(d.pressure / 100.0).toFixed(2)} hPa\n` +
      `QNH            = ${d.qnh.toFixed(2)} hPa\n` +
      `VARIO          = ${d.vario >= 0 ? '+' : ''}${d.vario.toFixed(2)} m/s\n` +
      `VERTICAL ACCEL = ${d.earthZAccel >= 0 ? '+' : ''}${d.earthZAccel.toFixed(3)} m/s²\n` +
      `KALMAN BIAS    = ${d.kalmanBias >= 0 ? '+' : ''}${d.kalmanBias.toFixed(4)} m/s²\n` +
      `PITCH          = ${d.pitch.toFixed(1)}°\n` +
      `ROLL           = ${d.roll.toFixed(1)}°\n` +
      `BARO OK        = ${d.baroOk}\n` +
      `IMU OK         = ${d.imuOk}\n` +
      `VARIO KAYNAGI  = ${d.imuFusionActive ? "IMU + barometre" : "Barometre"}`;

  } catch(e) {
    document.getElementById('tele').textContent =
      "STATUS HATASI: " + e;
    console.log("Status hatasi:", e);
  }

  setTimeout(refreshTele, 500);
}

async function loadParameters(){
  try {
    const r = await fetch('/status');
    const d = await r.json();

    document.getElementById('alpha').value  = d.alpha;
    document.getElementById('kfA').value    = d.kfA;
    document.getElementById('kfAB').value   = d.kfAB;
    document.getElementById('kfBaro').value = d.kfBaro;
    document.getElementById('qnh').value    = d.qnh;

    document.getElementById('deadband').value  = d.deadband;
    document.getElementById('climbMax').value  = d.climbMax;
    document.getElementById('sinkAlarm').value = d.sinkAlarm;
    document.getElementById('toneMin').value   = d.toneMin;
    document.getElementById('toneMax').value   = d.toneMax;
    document.getElementById('sinkTone').value  = d.sinkTone;

  } catch(e) {
    console.log("Parametre yukleme hatasi:", e);
  }
}
async function refreshLog(){
  try{const r=await fetch('/log');const t=await r.text();
  const p=document.getElementById('log');p.textContent=t;p.scrollTop=p.scrollHeight;}catch(e){}
  setTimeout(refreshLog,500);
}
async function applyFusion(){
  const q = new URLSearchParams({
    alpha: document.getElementById('alpha').value,
    kfA: document.getElementById('kfA').value,
    kfAB: document.getElementById('kfAB').value,
    kfBaro: document.getElementById('kfBaro').value,
    qnh: document.getElementById('qnh').value
  });

  try {
    const r = await fetch('/setFusion?' + q.toString());
    const d = await r.json();
    if (!r.ok || !d.ok) throw new Error(d.error || ("HTTP " + r.status));
    console.log("Fusion:", d);
    if(d.qnhChanged){
      trendPoints=[]; lastTrendT=0; drawTrend();
    }
  } catch(e) {
    alert("Fusion ayar hatasi: " + e.message);
  }
}
async function applyAudio(){
  const q = new URLSearchParams({
    deadband: document.getElementById('deadband').value,
    climbMax: document.getElementById('climbMax').value,
    sinkAlarm: document.getElementById('sinkAlarm').value,
    toneMin: document.getElementById('toneMin').value,
    toneMax: document.getElementById('toneMax').value,
    sinkTone: document.getElementById('sinkTone').value
  });

  try {
    const r = await fetch('/setAudio?' + q.toString());
    const t = await r.text();
    if (!r.ok) throw new Error(t);
    console.log("Audio:", t);
  } catch(e) {
    alert("Audio ayar hatasi: " + e.message);
  }
}

let trendPoints = [];
let lastTrendT = 0;
let trendCapacity = 1800;

function drawTrend(){
  const c = document.getElementById('trendCanvas');
  if (!c) return;
  const ctx = c.getContext('2d');
  const w = c.width, h = c.height;
  const ml=58, mr=58, mt=24, mb=34;
  const pw=w-ml-mr, ph=h-mt-mb;

  ctx.fillStyle='#000'; ctx.fillRect(0,0,w,h);
  ctx.font='12px monospace';

  if (trendPoints.length < 2) {
    ctx.fillStyle='#888';
    ctx.fillText('Trend icin veri birikiyor...', ml, mt+20);
    return;
  }

  let amin=Infinity, amax=-Infinity, vmin=Infinity, vmax=-Infinity;
  for (const p of trendPoints) {
    amin=Math.min(amin,p[1]); amax=Math.max(amax,p[1]);
    vmin=Math.min(vmin,p[2]); vmax=Math.max(vmax,p[2]);
  }
  let apad=Math.max(1.0,(amax-amin)*0.12);
  amin-=apad; amax+=apad;
  let vabs=Math.max(0.5,Math.abs(vmin),Math.abs(vmax))*1.15;
  vmin=-vabs; vmax=vabs;

  ctx.strokeStyle='#222'; ctx.lineWidth=1;
  ctx.fillStyle='#888';
  for(let i=0;i<=5;i++){
    const y=mt+ph*i/5;
    ctx.beginPath();ctx.moveTo(ml,y);ctx.lineTo(w-mr,y);ctx.stroke();
    const av=(amax-(amax-amin)*i/5).toFixed(1);
    const vv=(vmax-(vmax-vmin)*i/5).toFixed(1);
    ctx.fillText(av,4,y+4);
    ctx.fillText(vv,w-mr+8,y+4);
  }

  const t0=trendPoints[0][0], t1=trendPoints[trendPoints.length-1][0];
  const span=Math.max(1,t1-t0);
  for(let i=0;i<=5;i++){
    const x=ml+pw*i/5;
    ctx.strokeStyle='#222';
    ctx.beginPath();ctx.moveTo(x,mt);ctx.lineTo(x,h-mb);ctx.stroke();
    const sec=(span*i/5)/1000;
    const label=(sec/60).toFixed(sec>=600?0:1)+'m';
    ctx.fillStyle='#888';
    ctx.fillText(label,x-12,h-10);
  }

  function plot(idx, minv, maxv, color){
    ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
    let first=true;
    for(const p of trendPoints){
      const x=ml+((p[0]-t0)/span)*pw;
      const y=mt+(maxv-p[idx])/(maxv-minv)*ph;
      if(first){ctx.moveTo(x,y);first=false;} else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }

  plot(1,amin,amax,'#00d8ff');
  plot(2,vmin,vmax,'#ffb000');

  ctx.fillStyle='#00d8ff'; ctx.fillText('Irtifa (m)',ml,15);
  ctx.fillStyle='#ffb000'; ctx.fillText('Vario (m/s)',ml+110,15);
  ctx.fillStyle='#888'; ctx.fillText('gecen sure',w/2-35,h-10);
}

async function refreshTrend(){
  try{
    const r=await fetch('/trend?since='+lastTrendT,{cache:'no-store'});
    if(!r.ok) throw new Error('HTTP '+r.status);
    const d=await r.json();
    trendCapacity=d.capacity || trendCapacity;

    if(d.reset){
      trendPoints=[];
      lastTrendT=0;
    }

    if(Array.isArray(d.points) && d.points.length){
      for(const p of d.points){
        if(lastTrendT===0 || p[0]>lastTrendT){
          trendPoints.push(p);
          lastTrendT=p[0];
        }
      }
      if(trendPoints.length>trendCapacity)
        trendPoints=trendPoints.slice(trendPoints.length-trendCapacity);
    }

    document.getElementById('trendInfo').textContent =
      `${trendPoints.length}/${trendCapacity} nokta, 1 Hz, ` +
      `${d.memory || '?'}; irtifa + vario RAM trendi`;
    drawTrend();
  }catch(e){
    document.getElementById('trendInfo').textContent='Trend hatasi: '+e;
  }
  setTimeout(refreshTrend,2000);
}

async function calibrateAltitude(){
  const el=document.getElementById('knownAltitudeMeters');
  const out=document.getElementById('calResult');
  const entered=el.value.trim().replace(',', '.');
  const alt=Number(entered);
  if(!/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)$/.test(entered) ||
     !Number.isFinite(alt) || alt < -500 || alt > 9000){
    out.className='err'; out.textContent='Rakimi metre olarak -500 ile 9000 arasinda gir (orn. 975 veya 975,5).';
    return;
  }
  try{
    out.className='note'; out.textContent='Kalibre ediliyor...';
    const r=await fetch('/calibrateAltitude?alt='+encodeURIComponent(alt));
    const d=await r.json();
    if(!r.ok || !d.ok) throw new Error(d.error || ('HTTP '+r.status));
    document.getElementById('qnh').value=d.qnh.toFixed(2);
    out.className='ok';
    out.textContent=`Kalibre edildi: ${d.altitude.toFixed(1)} m, `+
                    `P=${(d.pressure/100).toFixed(2)} hPa, QNH=${d.qnh.toFixed(2)} hPa`;
    trendPoints=[]; lastTrendT=0; drawTrend();
    setTimeout(loadParameters,300);
  }catch(e){
    out.className='err'; out.textContent='Kalibrasyon hatasi: '+e;
  }
}

loadParameters();
refreshTele();
refreshLog();
refreshTrend();
</script>
</body></html>
)HTMLPAGE";

    void handleRoot() {
        server.sendHeader("Cache-Control", "no-store");
        server.send_P(200, "text/html", PAGE);
    }

    void handleLog() {
        server.send(200, "text/plain", DebugLog::snapshot());
    }

void handleStatus() {
    VarioState s = SharedState::read();
    Tunables t = TunablesStore::read();

    char buf[900];

    snprintf(buf, sizeof(buf),
        "{"
        "\"alt\":%.2f,"
        "\"vario\":%.2f,"
        "\"earthZAccel\":%.3f,"
        "\"baroAlt\":%.2f,"
        "\"pressure\":%.1f,"
        "\"kalmanBias\":%.4f,"
        "\"pitch\":%.1f,"
        "\"roll\":%.1f,"
        "\"baroOk\":%d,"
        "\"imuOk\":%d,"
        "\"imuCalibrated\":%d,"
        "\"imuFusionActive\":%d,"
        "\"imuCalibrationStatus\":%u,"
        "\"alpha\":%.3f,"
        "\"kfA\":%.3f,"
        "\"kfAB\":%.5f,"
        "\"kfBaro\":%.4f,"
        "\"qnh\":%.2f,"
        "\"deadband\":%.2f,"
        "\"climbMax\":%.2f,"
        "\"sinkAlarm\":%.2f,"
        "\"toneMin\":%d,"
        "\"toneMax\":%d,"
        "\"sinkTone\":%d"
        "}",
        
        s.altitudeM,
        s.climbRateMps,
        s.earthZAccelMps2,
        s.baroAltitudeM,
        s.pressurePa,
        s.kalmanAccelBias,
        s.pitchDeg,
        s.rollDeg,
        s.baroOk,
        s.imuOk,
        s.imuCalibrated,
        s.imuFusionActive,
        (unsigned)s.imuCalibrationStatus,

        t.fusion.compFilterAlpha,
        t.fusion.kfAccelVar,
        t.fusion.kfAccelBiasVar,
        t.fusion.kfBaroVar,
        t.fusion.qnhHpa,

        t.audio.climbDeadbandMps,
        t.audio.climbMaxMps,
        t.audio.sinkAlarmMps,
        t.audio.toneMinHz,
        t.audio.toneMaxHz,
        t.audio.sinkToneHz
    );

    server.send(200, "application/json", buf);
}

    // Reject empty, malformed and non-finite numbers before String conversion.
    bool validNumericArgs() {
        for (int i = 0; i < server.args(); ++i) {
            const String value = server.arg(i);
            char* end = nullptr;
            const float parsed = strtof(value.c_str(), &end);
            const String name = server.argName(i);
            const bool tone = name == "toneMin" || name == "toneMax" || name == "sinkTone";
            if (end == value.c_str() || *end != '\0' || !isfinite(parsed) ||
                (tone && (parsed < 150 || parsed > 5000 || floorf(parsed) != parsed))) {
                server.send(400, "application/json", "{\"ok\":false,\"error\":\"Gecersiz sayisal parametre\"}");
                return false;
            }
        }
        return true;
    }

    void handleCalibrateImu() {
        const VarioState s = SharedState::read();
        if (!s.imuOk) {
            server.send(503, "application/json", "{\"error\":\"IMU olcumu yok\"}");
            return;
        }
        if (s.imuCalibrationStatus == 1 || !requestGroundImuCalibration()) {
            server.send(409, "application/json", "{\"error\":\"Kalibrasyon zaten suruyor\"}");
            return;
        }
        server.send(202, "application/json", "{\"ok\":true}");
    }

    void handleSetFusion() {
        if (!validNumericArgs()) return;
        Tunables t = TunablesStore::read();
        const float oldQnh = t.fusion.qnhHpa;
        if (server.hasArg("alpha"))  t.fusion.compFilterAlpha = server.arg("alpha").toFloat();
        if (server.hasArg("kfA"))    t.fusion.kfAccelVar      = server.arg("kfA").toFloat();
        if (server.hasArg("kfAB"))   t.fusion.kfAccelBiasVar  = server.arg("kfAB").toFloat();
        if (server.hasArg("kfBaro")) t.fusion.kfBaroVar       = server.arg("kfBaro").toFloat();
        if (server.hasArg("qnh"))    t.fusion.qnhHpa          = server.arg("qnh").toFloat();

        if (!TunablesStore::validFusion(t.fusion)) {
            server.send(400, "application/json", "{\"ok\":false,\"error\":\"Fuzyon parametreleri aralik disi\"}");
            return;
        }

        const bool qnhChanged = fabsf(t.fusion.qnhHpa - oldQnh) > 0.001f;
        TunablesStore::writeFusion(t.fusion);
        if (qnhChanged) TrendBuffer::clear();

        DebugLog::logf(">> web: fuzyon guncellendi alpha=%.3f kfA=%.3f kfAB=%.5f kfBaro=%.4f qnh=%.2f\n",
                        t.fusion.compFilterAlpha, t.fusion.kfAccelVar, t.fusion.kfAccelBiasVar,
                        t.fusion.kfBaroVar, t.fusion.qnhHpa);

        char reply[96];
        snprintf(reply, sizeof(reply), "{\"ok\":true,\"qnhChanged\":%d,\"qnh\":%.2f}",
                 qnhChanged ? 1 : 0, t.fusion.qnhHpa);
        server.send(200, "application/json", reply);
    }


    void handleCalibrateAltitude() {
        if (!validNumericArgs()) return;
        if (!server.hasArg("alt")) {
            server.send(400, "application/json", "{\"ok\":false,\"error\":\"alt parametresi eksik\"}");
            return;
        }

        const float knownAltM = server.arg("alt").toFloat();
        const VarioState s = SharedState::read();

        if (!isfinite(knownAltM) || knownAltM < -500.0f || knownAltM > 9000.0f) {
            server.send(400, "application/json", "{\"ok\":false,\"error\":\"rakim aralik disi\"}");
            return;
        }
        if (!s.baroOk || !isfinite(s.pressurePa) || s.pressurePa < 20000.0f || s.pressurePa > 120000.0f) {
            server.send(503, "application/json", "{\"ok\":false,\"error\":\"gecerli barometre basinci yok\"}");
            return;
        }

        const float base = 1.0f - knownAltM / 44330.0f;
        if (base <= 0.0f) {
            server.send(400, "application/json", "{\"ok\":false,\"error\":\"rakim formulu icin gecersiz\"}");
            return;
        }

        const float qnhHpa =
            s.pressurePa / (100.0f * powf(base, 1.0f / 0.190295f));

        if (!isfinite(qnhHpa) || qnhHpa < 800.0f || qnhHpa > 1100.0f) {
            server.send(400, "application/json", "{\"ok\":false,\"error\":\"hesaplanan QNH mantiksiz; rakim/basinc kontrol et\"}");
            return;
        }

        Tunables t = TunablesStore::read();
        t.fusion.qnhHpa = qnhHpa;
        TunablesStore::writeFusion(t.fusion);

        // Old altitude history used a different absolute reference.
        TrendBuffer::clear();

        DebugLog::logf(">> web: irtifa kalibrasyonu knownAlt=%.2fm P=%.1fPa -> QNH=%.2fhPa\n",
                       knownAltM, s.pressurePa, qnhHpa);

        char reply[180];
        snprintf(reply, sizeof(reply),
                 "{\"ok\":true,\"altitude\":%.2f,\"pressure\":%.1f,\"qnh\":%.2f}",
                 knownAltM, s.pressurePa, qnhHpa);
        server.send(200, "application/json", reply);
    }

    void handleTrend() {
        const size_t nAvail = TrendBuffer::count();
        const size_t bytes = (nAvail ? nAvail : 1) * sizeof(TrendPoint);

        TrendPoint* snap = static_cast<TrendPoint*>(
            heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
        if (!snap) {
            snap = static_cast<TrendPoint*>(
                heap_caps_malloc(bytes, MALLOC_CAP_8BIT)
            );
        }
        if (!snap) {
            server.send(503, "application/json", "{\"error\":\"trend snapshot icin RAM yok\"}");
            return;
        }

        const size_t n = TrendBuffer::snapshot(snap, nAvail);
        uint32_t since = 0;
        if (server.hasArg("since")) {
            since = strtoul(server.arg("since").c_str(), nullptr, 10);
        }

        server.sendHeader("Cache-Control", "no-store");
        server.setContentLength(CONTENT_LENGTH_UNKNOWN);
        server.send(200, "application/json", "");

        char head[160];
        snprintf(head, sizeof(head),
                 "{\"capacity\":%u,\"count\":%u,\"memory\":\"%s\",\"points\":[",
                 (unsigned)TrendBuffer::capacity(), (unsigned)n,
                 TrendBuffer::usingPsram() ? "PSRAM" : "internal RAM");
        server.sendContent(head);

        String chunk;
        chunk.reserve(1200);
        bool first = true;
        for (size_t i = 0; i < n; ++i) {
            const TrendPoint& p = snap[i];
            if (since != 0 && (int32_t)(p.tMs - since) <= 0) continue;

            char item[96];
            snprintf(item, sizeof(item),
                     "%s[%lu,%.2f,%.3f,%.1f]",
                     first ? "" : ",",
                     (unsigned long)p.tMs,
                     p.altitudeM,
                     p.varioMps,
                     p.pressurePa);
            first = false;

            if (chunk.length() + strlen(item) > 1000) {
                server.sendContent(chunk);
                chunk = "";
            }
            chunk += item;
        }
        if (chunk.length()) server.sendContent(chunk);
        server.sendContent("]}");
        server.sendContent("");

        heap_caps_free(snap);
    }

    void handleSetAudio() {
        if (!validNumericArgs()) return;
        Tunables t = TunablesStore::read();
        if (server.hasArg("deadband"))  t.audio.climbDeadbandMps = server.arg("deadband").toFloat();
        if (server.hasArg("climbMax"))  t.audio.climbMaxMps      = server.arg("climbMax").toFloat();
        if (server.hasArg("sinkAlarm")) t.audio.sinkAlarmMps     = server.arg("sinkAlarm").toFloat();
        if (server.hasArg("toneMin"))   t.audio.toneMinHz        = (int)server.arg("toneMin").toFloat();
        if (server.hasArg("toneMax"))   t.audio.toneMaxHz        = (int)server.arg("toneMax").toFloat();
        if (server.hasArg("sinkTone"))  t.audio.sinkToneHz       = (int)server.arg("sinkTone").toFloat();
        if (!TunablesStore::validAudio(t.audio)) {
            server.send(400, "text/plain", "Ses parametreleri aralik disi");
            return;
        }
        TunablesStore::writeAudio(t.audio);
        DebugLog::log(">> web: ses parametreleri guncellendi\n");
        server.send(200, "text/plain", "OK");
    }
}

void webTaskFunc(void* /*pvParameters*/) {
    // WiFi.softAP()'nin cektigi ani akim darbesi, MS5607/MPU6050
    // begin()'inin hassas I2C/guc penceresiyle cakisirsa sensor init'i
    // bozabiliyor (bkz. dosya basindaki guc dekuplaj notu). Bu yuzden
    // AP'yi ancak vario task sensor init'ini bitirdikten sonra aciyoruz.
    // 8s timeout: sensor init hic basarili olmasa da (ornegin baro/imu
    // fiziksel olarak takili degilse) WiFi'siz sonsuza kalinmasin.
    BootSync::waitForSensorsReady(8000);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);

    server.on("/", handleRoot);
    server.on("/log", handleLog);
    server.on("/status", handleStatus);
    server.on("/setFusion", handleSetFusion);
    server.on("/calibrateAltitude", handleCalibrateAltitude);
    server.on("/calibrateImu", HTTP_POST, handleCalibrateImu);
    server.on("/trend", handleTrend);
    server.on("/setAudio", handleSetAudio);
    server.begin();

    DebugLog::logf("WiFi AP baslatildi: %s / %s   http://%s\n",
                    AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());

    const TickType_t period = pdMS_TO_TICKS(20); // 50Hz poll; HTTP itself is event-driven
    TickType_t lastWake = xTaskGetTickCount();
    const uint32_t apStartMs = millis();

    bool     everConnected      = false; // en az bir kez istemci baglandi mi
    bool     trackingDisconnect = false; // az once bagliydi, simdi kimse yok, sayiyoruz
    uint32_t disconnectedAtMs   = 0;

    // Serial mirror: DebugLog'un tek Serial tuketicisi burasi. Bilerek
    // non-blocking — host tarafi okumuyorsa (baglanti kesildi/DTR dustu)
    // bu turu tamamen atlariz, ASLA Serial.print() ile bloklanmayiz.
    // Debug amacli oldugu icin bu sartta birkac satirin kaybolmasi kabul
    // edilebilir; onemli olan web_task'in (ve dolayisiyla vario_task'in,
    // her ikisi de bu bloktan tamamen bagimsiz) asla kilitlenmemesi.
    uint32_t lastSerialMirrorMs = 0;

    auto shutdownWifi = [](const char* reason) {
        DebugLog::logf("%s — AP kapatiliyor.\n", reason);
        server.stop();
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_OFF); // radyo tamamen kapanir, guc tasarrufu + I2C/guc hatti uzerindeki WiFi paraziti tamamen biter
        // Bu task'in artik yapacak isi yok — tamamen askiya al. Tekrar
        // acmak icin cihazi resetlemek/guc vermek gerekir.
        vTaskSuspend(nullptr);
    };

    for (;;) {
        server.handleClient();

        uint8_t  stationCount = WiFi.softAPgetStationNum();
        uint32_t nowMs        = millis();

        if (Serial && nowMs - lastSerialMirrorMs >= 200) {
            lastSerialMirrorMs = nowMs;
            String chunk = DebugLog::pullNewForSerial();
            if (chunk.length() && (size_t)Serial.availableForWrite() >= chunk.length()) {
                Serial.print(chunk);
            }
            // Yer yoksa: bu turu sessizce atla, DebugLog buffer'i ve
            // /log web sayfasi zaten etkilenmedi — sadece Serial'e
            // yansimadi.
        }

        if (stationCount > 0) {
            everConnected      = true;
            trackingDisconnect = false; // hala/tekrar bagli
        } else if (everConnected && !trackingDisconnect) {
            // Az once en az bir istemci vardi, simdi hic yok -> ayrilma
            // anini kaydet, tolerans suresini burada baslat.
            trackingDisconnect = true;
            disconnectedAtMs   = nowMs;
        }

        if (!everConnected) {
            // Hic kimse baglanmadi — sonsuza kadar bekletmeyip vazgec.
            if (nowMs - apStartMs >= Timing::WIFI_ON_DURATION_MS) {
                shutdownWifi("Zaman asimi: kimse baglanmadi");
            }
        } else if (trackingDisconnect) {
            // Baglıydı, ayrildi — kisa tolerans sonunda hala kimse yoksa kapat.
            if (nowMs - disconnectedAtMs >= Timing::WIFI_DISCONNECT_GRACE_MS) {
                shutdownWifi("Istemci baglantiyi kesti");
            }
        }

        vTaskDelayUntil(&lastWake, period);
    }
}
#endif // VARIO_USE_WIFI
