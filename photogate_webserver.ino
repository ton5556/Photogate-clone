#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ================= PIN =================
#define PHOTOGATE_PIN 4
#define BTN_MENU      5
#define BTN_ENTER     7
#define BTN_GATE      6
#define LED_PIN       13

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================= CONFIG =================
#define MAX_GATES    20
#define MIN_GATES     2
#define DEBOUNCE_US  1000
#define TIMEOUT_MS   30000UL

// ================= WiFi =================
// *** แก้ตรงนี้ให้ตรงกับ WiFi ที่ใช้ ***
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// ถ้าไม่มี WiFi ให้ ESP32 เปิดเป็น Access Point แทน
// เชื่อมต่อที่ SSID: "Photogate" รหัส: "12345678"
// แล้วเปิด http://192.168.4.1
#define USE_AP_FALLBACK true

WebServer server(80);

// ================= DATA =================
volatile uint32_t gateTimes[MAX_GATES];
volatile uint8_t  currentGate     = 0;
volatile bool     measurementDone = false;
volatile uint32_t lastTriggerTime = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================= STATE =================
int           numGates            = 10;
bool          systemActive        = false;
bool          measurementComplete = false;
unsigned long measurementStartTime = 0;
bool          armed               = false;

// ================= MENU =================
enum AppMenuState { MENU_MAIN, MENU_MEASURE, MENU_VIEW };
AppMenuState menuState = MENU_MAIN;
int          menuIndex = 0;

// ================= DISPLAY =================
int           lastMenuIndex    = -1;
int           lastNumGates     = -1;
int           lastGateCount    = -1;
int           displayGate      = 0;
unsigned long lastDisplayUpdate = 0;

// ================= SERIAL =================
String serialBuffer = "";

// ===============================================================
// TYPES  (นิยามก่อนทุกอย่าง เพราะ forward declaration ต้องใช้)
// ===============================================================

enum ButtonEvent { BTN_NONE, BTN_MENU_EVENT, BTN_ENTER_EVENT, BTN_GATE_EVENT };

struct PhysicsResult {
  float   t_s[MAX_GATES];
  float   s_m[MAX_GATES];
  float   v_ms[MAX_GATES];
  float   a_ms2[MAX_GATES];
  float   dt_s[MAX_GATES];
  uint8_t n, nv, na;
  float   v_avg, v_max, v_min, a_avg;
  float   totalTime_s, totalDist_m;
};

// ===============================================================
// FORWARD DECLARATIONS
// ===============================================================
void          resetSystem();
void          startMeasurement();
void          displayMenu();
void          runMeasurement();
void          displayIntervals();
void          showAllIntervals();
void          handleButtonEvent(ButtonEvent btn);
void          handleSerialCommand();
void          processSerialCommand(String cmd);
PhysicsResult calcPhysics(uint32_t* times, uint8_t n, float sep);

// ===============================================================
// BUTTON EVENTS
// ===============================================================

ButtonEvent readButtons() {
  static bool lastMenuBtnState = HIGH, lastEnterState = HIGH, lastGateState = HIGH;
  static unsigned long lastMenuTime = 0, lastEnterTime = 0, lastGateTime = 0;

  bool menuBtnState = digitalRead(BTN_MENU);
  bool enterState   = digitalRead(BTN_ENTER);
  bool gateState    = digitalRead(BTN_GATE);

  if (menuBtnState == LOW && lastMenuBtnState == HIGH && millis() - lastMenuTime > 200) {
    lastMenuTime = millis(); lastMenuBtnState = menuBtnState; return BTN_MENU_EVENT;
  }
  if (enterState == LOW && lastEnterState == HIGH && millis() - lastEnterTime > 200) {
    lastEnterTime = millis(); lastEnterState = enterState; return BTN_ENTER_EVENT;
  }
  if (gateState == LOW && lastGateState == HIGH && millis() - lastGateTime > 200) {
    lastGateTime = millis(); lastGateState = gateState; return BTN_GATE_EVENT;
  }

  lastMenuBtnState = menuBtnState;
  lastEnterState   = enterState;
  lastGateState    = gateState;
  return BTN_NONE;
}

// ===============================================================
// ISR
// ===============================================================
void IRAM_ATTR photogateISR() {
  uint32_t now = micros();
  if (!armed) return;
  if (now - lastTriggerTime < DEBOUNCE_US) return;

  portENTER_CRITICAL_ISR(&mux);
  lastTriggerTime = now;
  if (systemActive && currentGate < numGates) {
    gateTimes[currentGate++] = now;
    if (currentGate >= numGates) {
      measurementDone = true;
      systemActive    = false;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

// ===============================================================
// PHYSICS HELPERS
// ===============================================================

float gateSeparation_m = 0.10;   // default 10 cm — ตั้งได้จาก Web UI หรือ Serial SEP

PhysicsResult calcPhysics(uint32_t* times, uint8_t n, float sep) {
  PhysicsResult r;
  memset(&r, 0, sizeof(r));
  r.n  = n;
  r.nv = (n > 1) ? n - 1 : 0;
  r.na = (n > 2) ? n - 2 : 0;

  if (n < 2) return r;

  // t สะสม (วินาที)
  r.t_s[0] = 0.0f;
  for (int i = 1; i < n; i++) {
    r.dt_s[i-1] = (times[i] - times[i-1]) / 1e6f;
    r.t_s[i]    = r.t_s[i-1] + r.dt_s[i-1];
  }
  r.totalTime_s = r.t_s[n-1];

  // s สะสม (เมตร)
  r.s_m[0] = 0.0f;
  for (int i = 1; i < n; i++) {
    r.s_m[i] = sep * i;
  }
  r.totalDist_m = r.s_m[n-1];

  // v เฉลี่ยแต่ละช่วง  v_i = sep / dt_i
  r.v_max = -1e9f; r.v_min = 1e9f;
  float vSum = 0;
  for (int i = 0; i < r.nv; i++) {
    r.v_ms[i] = (r.dt_s[i] > 0) ? (sep / r.dt_s[i]) : 0;
    vSum += r.v_ms[i];
    if (r.v_ms[i] > r.v_max) r.v_max = r.v_ms[i];
    if (r.v_ms[i] < r.v_min) r.v_min = r.v_ms[i];
  }
  r.v_avg = vSum / r.nv;

  // a เฉลี่ยแต่ละช่วง  a_i = (v_{i+1} - v_i) / ((dt_i + dt_{i+1}) / 2)
  float aSum = 0;
  for (int i = 0; i < r.na; i++) {
    float dt_avg = (r.dt_s[i] + r.dt_s[i+1]) / 2.0f;
    r.a_ms2[i]   = (dt_avg > 0) ? (r.v_ms[i+1] - r.v_ms[i]) / dt_avg : 0;
    aSum += r.a_ms2[i];
  }
  r.a_avg = (r.na > 0) ? aSum / r.na : 0;

  return r;
}

// ===============================================================
// HTML PAGE  (เก็บใน PROGMEM เพื่อประหยัด RAM)
// ===============================================================
const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="th">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Photogate Lab</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {
    --bg:#0d1117; --surface:#161b22; --border:#30363d;
    --accent:#58a6ff; --accent2:#3fb950; --accent3:#f78166;
    --accent4:#d2a8ff; --text:#e6edf3; --muted:#8b949e;
  }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text); font-family:'Segoe UI',system-ui,sans-serif; min-height:100vh; }

  header { background:var(--surface); border-bottom:1px solid var(--border);
    padding:16px 24px; display:flex; align-items:center; gap:12px; }
  header h1 { font-size:1.2rem; font-weight:600; }
  .badge { background:#21262d; border:1px solid var(--border); border-radius:20px;
    padding:2px 10px; font-size:.75rem; color:var(--muted); }
  #statusBadge { margin-left:auto; padding:4px 14px; border-radius:20px;
    font-size:.8rem; font-weight:600; }
  .status-idle    { background:#21262d; color:var(--muted); border:1px solid var(--border); }
  .status-ready   { background:#1a3a2a; color:var(--accent2); border:1px solid var(--accent2); }
  .status-live    { background:#3a1a1a; color:var(--accent3); border:1px solid var(--accent3); animation:pulse 1s infinite; }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.5} }

  .container { max-width:1100px; margin:0 auto; padding:24px 16px; }

  .control-bar { display:flex; flex-wrap:wrap; gap:10px; margin-bottom:24px; align-items:center; }
  .control-bar label { font-size:.85rem; color:var(--muted); }
  input[type=number] { background:#21262d; border:1px solid var(--border); color:var(--text);
    border-radius:6px; padding:6px 10px; width:90px; font-size:.9rem; }
  button { border:none; border-radius:6px; padding:8px 18px; font-size:.85rem;
    font-weight:600; cursor:pointer; transition:opacity .15s; }
  button:hover { opacity:.8; }
  .btn-primary   { background:var(--accent);  color:#0d1117; }
  .btn-success   { background:var(--accent2); color:#0d1117; }
  .btn-danger    { background:var(--accent3); color:#0d1117; }
  .btn-secondary { background:#21262d; color:var(--text); border:1px solid var(--border); }

  .summary-grid { display:grid; grid-template-columns:repeat(auto-fill,minmax(170px,1fr)); gap:12px; margin-bottom:24px; }
  .summary-card { background:var(--surface); border:1px solid var(--border); border-radius:10px; padding:14px 16px; }
  .summary-card .label { font-size:.72rem; color:var(--muted); text-transform:uppercase; letter-spacing:.05em; margin-bottom:4px; }
  .summary-card .value { font-size:1.4rem; font-weight:700; }
  .summary-card .unit  { font-size:.75rem; color:var(--muted); margin-left:3px; }

  .charts-grid { display:grid; grid-template-columns:1fr 1fr; gap:16px; margin-bottom:24px; }
  @media(max-width:700px){ .charts-grid{ grid-template-columns:1fr; } }
  .chart-card { background:var(--surface); border:1px solid var(--border); border-radius:10px; padding:16px; }
  .chart-card h3 { font-size:.85rem; color:var(--muted); margin-bottom:12px; font-weight:500; }
  .chart-card canvas { max-height:220px; }

  .chart-full { background:var(--surface); border:1px solid var(--border); border-radius:10px;
    padding:16px; margin-bottom:24px; }
  .chart-full h3 { font-size:.85rem; color:var(--muted); margin-bottom:12px; font-weight:500; }
  .chart-full canvas { max-height:250px; }

  .table-wrap { background:var(--surface); border:1px solid var(--border); border-radius:10px;
    overflow-x:auto; margin-bottom:24px; }
  table { width:100%; border-collapse:collapse; font-size:.85rem; }
  th { background:#21262d; padding:10px 14px; text-align:left; color:var(--muted);
    font-weight:600; font-size:.75rem; text-transform:uppercase; letter-spacing:.05em;
    border-bottom:1px solid var(--border); }
  td { padding:9px 14px; border-bottom:1px solid var(--border); font-variant-numeric:tabular-nums; }
  tr:last-child td { border-bottom:none; }
  tr:hover td { background:#1c2128; }

  .no-data { text-align:center; padding:60px; color:var(--muted); }
  .no-data .icon { font-size:3rem; margin-bottom:12px; }

  footer { text-align:center; padding:24px; font-size:.75rem; color:var(--muted);
    border-top:1px solid var(--border); }
</style>
</head>
<body>
<header>
  <span style="font-size:1.4rem">⚡</span>
  <h1>Photogate Physics Lab</h1>
  <span class="badge">ESP32-S3</span>
  <div id="statusBadge" class="status-idle">Idle</div>
</header>

<div class="container">

  <!-- Control Bar -->
  <div class="control-bar">
    <label>Gate separation (m):</label>
    <input type="number" id="sepInput" value="0.10" step="0.01" min="0.001">
    <button class="btn-primary" onclick="fetchData()">🔄 Refresh</button>
    <button class="btn-success" onclick="autoRefresh()">▶ Auto (2s)</button>
    <button class="btn-secondary" onclick="stopAuto()">⏹ Stop</button>
    <button class="btn-danger"  onclick="resetDevice()">↺ Reset</button>
    <button class="btn-secondary" onclick="exportCSV()">⬇ Export CSV</button>
  </div>

  <!-- Summary Cards -->
  <div class="summary-grid" id="summaryGrid">
    <div class="no-data" style="grid-column:1/-1">
      <div class="icon">🔬</div>
      <div>ยังไม่มีข้อมูล — กด Refresh หรือ Start Measurement บนอุปกรณ์</div>
    </div>
  </div>

  <!-- Charts -->
  <div id="chartsSection" style="display:none">
    <div class="chart-full">
      <h3>📍 s–t  (Position vs Time)</h3>
      <canvas id="chartST"></canvas>
    </div>
    <div class="charts-grid">
      <div class="chart-card">
        <h3>🏃 v–t  (Velocity vs Time)</h3>
        <canvas id="chartVT"></canvas>
      </div>
      <div class="chart-card">
        <h3>🚀 a–t  (Acceleration vs Time)</h3>
        <canvas id="chartAT"></canvas>
      </div>
    </div>

    <!-- Data Table -->
    <div class="table-wrap">
      <table id="dataTable">
        <thead>
          <tr>
            <th>#</th>
            <th>Gate</th>
            <th>t (s)</th>
            <th>Δt (ms)</th>
            <th>s (m)</th>
            <th>v (m/s)</th>
            <th>a (m/s²)</th>
          </tr>
        </thead>
        <tbody id="tableBody"></tbody>
      </table>
    </div>
  </div>

</div>
<footer>Photogate Meter · By Tonkla SKDW19 · ESP32-S3</footer>

<script>
let charts = {};
let autoTimer = null;
let lastData = null;

const COLORS = {
  st: '#58a6ff',
  vt: '#3fb950',
  at: '#f78166',
  grid: '#21262d',
  text: '#8b949e'
};

Chart.defaults.color = COLORS.text;

function makeChart(id, label, xLabel, yLabel, color, data, fill=false) {
  const ctx = document.getElementById(id).getContext('2d');
  if (charts[id]) charts[id].destroy();
  charts[id] = new Chart(ctx, {
    type: 'line',
    data: {
      datasets: [{
        label: label,
        data: data,
        borderColor: color,
        backgroundColor: fill ? color + '22' : 'transparent',
        borderWidth: 2.5,
        pointRadius: 5,
        pointHoverRadius: 7,
        tension: 0.3,
        fill: fill
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: true,
      plugins: {
        legend: { display: false },
        tooltip: {
          callbacks: {
            label: ctx => ` ${ctx.parsed.y.toFixed(4)}`
          }
        }
      },
      scales: {
        x: {
          type: 'linear',
          title: { display: true, text: xLabel, color: COLORS.text },
          grid: { color: COLORS.grid },
          ticks: { color: COLORS.text }
        },
        y: {
          title: { display: true, text: yLabel, color: COLORS.text },
          grid: { color: COLORS.grid },
          ticks: { color: COLORS.text }
        }
      }
    }
  });
}

function renderSummary(p) {
  const grid = document.getElementById('summaryGrid');
  const cards = [
    { label:'Total Time',   value: p.totalTime_s.toFixed(4),  unit:'s',    color:'#58a6ff' },
    { label:'Total Dist',   value: p.totalDist_m.toFixed(4),  unit:'m',    color:'#3fb950' },
    { label:'Gates',        value: p.n,                        unit:'',     color:'#d2a8ff' },
    { label:'v avg',        value: p.v_avg.toFixed(4),         unit:'m/s',  color:'#3fb950' },
    { label:'v max',        value: p.v_max.toFixed(4),         unit:'m/s',  color:'#58a6ff' },
    { label:'v min',        value: p.v_min.toFixed(4),         unit:'m/s',  color:'#f78166' },
    { label:'a avg',        value: p.a_avg.toFixed(4),         unit:'m/s²', color:'#f78166' },
    { label:'Δv total',     value: (p.v_max - p.v_min).toFixed(4), unit:'m/s', color:'#d2a8ff' },
  ];
  grid.innerHTML = cards.map(c => `
    <div class="summary-card">
      <div class="label">${c.label}</div>
      <div class="value" style="color:${c.color}">${c.value}<span class="unit">${c.unit}</span></div>
    </div>`).join('');
}

function renderTable(p) {
  const tb = document.getElementById('tableBody');
  let rows = '';
  for (let i = 0; i < p.n; i++) {
    const v = i < p.nv ? p.v_ms[i].toFixed(5) : '—';
    const a = i < p.na ? p.a_ms2[i].toFixed(5) : '—';
    const dt = i < p.nv ? (p.dt_s[i] * 1000).toFixed(3) : '—';
    rows += `<tr>
      <td>${i+1}</td>
      <td>G${i+1}</td>
      <td>${p.t_s[i].toFixed(5)}</td>
      <td>${dt}</td>
      <td>${p.s_m[i].toFixed(4)}</td>
      <td>${v}</td>
      <td>${a}</td>
    </tr>`;
  }
  tb.innerHTML = rows;
}

function renderCharts(p) {
  // s-t
  const stData = p.t_s.slice(0, p.n).map((t,i) => ({ x: t, y: p.s_m[i] }));
  makeChart('chartST', 's (m)', 't (s)', 's (m)', COLORS.st, stData, true);

  // v-t  — plot at midpoint time
  const vtData = [];
  for (let i = 0; i < p.nv; i++) {
    const tmid = (p.t_s[i] + p.t_s[i+1]) / 2;
    vtData.push({ x: tmid, y: p.v_ms[i] });
  }
  makeChart('chartVT', 'v (m/s)', 't (s)', 'v (m/s)', COLORS.vt, vtData);

  // a-t  — plot at midpoint of the two intervals
  const atData = [];
  for (let i = 0; i < p.na; i++) {
    const tmid = (p.t_s[i] + p.t_s[i+2]) / 2;
    atData.push({ x: tmid, y: p.a_ms2[i] });
  }
  makeChart('chartAT', 'a (m/s²)', 't (s)', 'a (m/s²)', COLORS.at, atData);
}

async function fetchData() {
  const sep = parseFloat(document.getElementById('sepInput').value) || 0.10;
  try {
    const r = await fetch(`/api/data?sep=${sep}`);
    const d = await r.json();
    lastData = d;

    const badge = document.getElementById('statusBadge');
    if (d.systemActive) {
      badge.className = 'status-live'; badge.textContent = '● Measuring';
    } else if (d.measurementComplete) {
      badge.className = 'status-ready'; badge.textContent = '✓ Done';
    } else {
      badge.className = 'status-idle'; badge.textContent = 'Idle';
    }

    if (d.physics && d.physics.n >= 2) {
      document.getElementById('chartsSection').style.display = 'block';
      renderSummary(d.physics);
      renderCharts(d.physics);
      renderTable(d.physics);
    }
  } catch(e) {
    console.error(e);
  }
}

function exportCSV() {
  if (!lastData || !lastData.physics) return;
  const p = lastData.physics;
  let csv = 'gate,t_s,dt_ms,s_m,v_ms,a_ms2\n';
  for (let i = 0; i < p.n; i++) {
    const dt = i < p.nv ? (p.dt_s[i]*1000).toFixed(3) : '';
    const v  = i < p.nv ? p.v_ms[i].toFixed(5) : '';
    const a  = i < p.na ? p.a_ms2[i].toFixed(5) : '';
    csv += `${i+1},${p.t_s[i].toFixed(6)},${dt},${p.s_m[i].toFixed(5)},${v},${a}\n`;
  }
  const blob = new Blob([csv], { type:'text/csv' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob); a.download = 'photogate_data.csv'; a.click();
}

async function resetDevice() {
  if (!confirm('Reset อุปกรณ์?')) return;
  await fetch('/api/reset', { method:'POST' });
  document.getElementById('chartsSection').style.display = 'none';
  document.getElementById('summaryGrid').innerHTML =
    '<div class="no-data" style="grid-column:1/-1"><div class="icon">🔬</div><div>Reset แล้ว</div></div>';
}

function autoRefresh() {
  stopAuto();
  autoTimer = setInterval(fetchData, 2000);
  fetchData();
}
function stopAuto() {
  if (autoTimer) { clearInterval(autoTimer); autoTimer = null; }
}

// Initial load
fetchData();
</script>
</body>
</html>
)rawhtml";

// ===============================================================
// WEB SERVER HANDLERS
// ===============================================================

void handleRoot() {
  server.send_P(200, "text/html", HTML_PAGE);
}

void handleApiData() {
  // อ่าน sep จาก query string  /api/data?sep=0.10
  float sep = gateSeparation_m;
  if (server.hasArg("sep")) {
    float v = server.arg("sep").toFloat();
    if (v > 0) { sep = v; gateSeparation_m = v; }
  }

  // copy data ออกมาอย่างปลอดภัย
  uint32_t localTimes[MAX_GATES];
  uint8_t  localGate;
  portENTER_CRITICAL(&mux);
  localGate = currentGate;
  for (int i = 0; i < localGate; i++) localTimes[i] = gateTimes[i];
  portEXIT_CRITICAL(&mux);

  PhysicsResult p = calcPhysics(localTimes, localGate, sep);

  // สร้าง JSON ด้วย ArduinoJson
  StaticJsonDocument<4096> doc;

  doc["systemActive"]        = systemActive;
  doc["measurementComplete"] = measurementComplete;
  doc["armed"]               = armed;
  doc["numGates"]            = numGates;
  doc["gatesCaptured"]       = localGate;
  doc["gateSeparation_m"]    = sep;

  // raw timestamps
  JsonArray raw = doc.createNestedArray("rawTimes_us");
  for (int i = 0; i < localGate; i++) raw.add(localTimes[i]);

  // physics object
  JsonObject ph = doc.createNestedObject("physics");
  ph["n"]           = p.n;
  ph["nv"]          = p.nv;
  ph["na"]          = p.na;
  ph["totalTime_s"] = p.totalTime_s;
  ph["totalDist_m"] = p.totalDist_m;
  ph["v_avg"]       = p.v_avg;
  ph["v_max"]       = p.v_max;
  ph["v_min"]       = p.v_min;
  ph["a_avg"]       = p.a_avg;

  JsonArray jt  = ph.createNestedArray("t_s");
  JsonArray js  = ph.createNestedArray("s_m");
  JsonArray jv  = ph.createNestedArray("v_ms");
  JsonArray ja  = ph.createNestedArray("a_ms2");
  JsonArray jdt = ph.createNestedArray("dt_s");

  for (int i = 0; i < p.n;  i++) { jt.add(p.t_s[i]);  js.add(p.s_m[i]); }
  for (int i = 0; i < p.nv; i++) { jv.add(p.v_ms[i]); jdt.add(p.dt_s[i]); }
  for (int i = 0; i < p.na; i++) { ja.add(p.a_ms2[i]); }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleApiReset() {
  resetSystem();
  menuState = MENU_MAIN;
  menuIndex = 0;
  lcd.clear();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ===============================================================
// WiFi SETUP
// ===============================================================
void setupWiFi() {
  // รีเซ็ต WiFi stack ให้สะอาดก่อน (สำคัญมากสำหรับ ESP32-S3)
  WiFi.disconnect(true, true);
  delay(200);

  bool tryStation = (strcmp(WIFI_SSID, "YOUR_SSID") != 0);

  if (tryStation) {
    WiFi.mode(WIFI_STA);         // ต้องตั้ง mode ก่อน begin เสมอ
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    lcd.clear();
    lcd.print("Connecting WiFi");
    Serial.print("Connecting to "); Serial.println(WIFI_SSID);

    unsigned long t0 = millis();
    int dot = 0;
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
      delay(500);
      lcd.setCursor(dot % 16, 1);
      lcd.print(".");
      dot++;
      Serial.print(".");
    }
    Serial.println();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());

    lcd.clear();
    lcd.print("WiFi OK!");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(2500);

  } else {
    // AP Fallback
    Serial.println("WiFi failed -> starting AP...");
    WiFi.disconnect(true, true);
    delay(100);

    WiFi.mode(WIFI_AP);          // ต้องตั้ง mode ใหม่ชัดเจน
    delay(100);

    // กำหนด IP ตายตัว ไม่ให้ขึ้น "No Internet" บน iPhone
    IPAddress localIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(localIP, gateway, subnet);
    delay(100);

    // channel=6, hidden=false, max_connection=4
    bool apOK = WiFi.softAP("Photogate", "12345678", 6, 0, 4);
    delay(1000);                 // ESP32-S3 ต้องรอนานกว่านี้

    IPAddress apIP = WiFi.softAPIP();
    Serial.print("AP "); Serial.print(apOK ? "OK" : "FAIL");
    Serial.print(" IP: "); Serial.println(apIP);
    Serial.println("Connect: Photogate / 12345678");
    Serial.println("Open: http://192.168.4.1");

    lcd.clear();
    if (apOK) {
      lcd.print("AP:Photogate OK ");
      lcd.setCursor(0, 1);
      lcd.print(apIP);
    } else {
      lcd.print("WiFi Error!");
      lcd.setCursor(0, 1);
      lcd.print("Check board sel.");
    }
    delay(3000);
  }
}

// ===============================================================
// SETUP
// ===============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Photogate Meter ===");
  Serial.println("Type HELP for available commands");

  pinMode(PHOTOGATE_PIN, INPUT);
  pinMode(BTN_MENU,  INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_GATE,  INPUT_PULLUP);
  pinMode(LED_PIN,   OUTPUT);

  attachInterrupt(PHOTOGATE_PIN, photogateISR, FALLING);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print(" Photogate Meter");
  lcd.setCursor(0, 1); lcd.print("By Tonkla SKDW19");
  delay(1500);

  setupWiFi();

  // Register routes
  server.on("/",          handleRoot);
  server.on("/api/data",  handleApiData);
  server.on("/api/reset", HTTP_POST, handleApiReset);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("Web server started");
  lcd.clear();
}

// ===============================================================
// LOOP
// ===============================================================
void loop() {
  server.handleClient();

  ButtonEvent btn = readButtons();
  handleButtonEvent(btn);
  handleSerialCommand();

  // Arming
  if (systemActive && !armed) {
    if (digitalRead(PHOTOGATE_PIN) == HIGH) {
      armed = true;
      measurementStartTime = millis();
      lcd.clear(); lcd.print("Armed!");
      delay(300);  lcd.clear();
    }
  }

  // Timeout
  if (systemActive && armed && !measurementDone) {
    if (millis() - measurementStartTime > TIMEOUT_MS) {
      systemActive = false;
      digitalWrite(LED_PIN, LOW);
      lcd.clear(); lcd.print("Timeout!");
      lcd.setCursor(0,1);
      uint8_t localGate;
      portENTER_CRITICAL(&mux); localGate = currentGate; portEXIT_CRITICAL(&mux);
      lcd.print("Got "); lcd.print(localGate); lcd.print("/"); lcd.print(numGates);
      delay(1500);
      if (localGate >= 2) measurementDone = true;
      else { menuState = MENU_MAIN; lcd.clear(); }
    }
  }

  switch (menuState) {
    case MENU_MAIN:    displayMenu();      break;
    case MENU_MEASURE: runMeasurement();   break;
    case MENU_VIEW:    displayIntervals(); break;
  }

  if (measurementDone && !measurementComplete) {
    showAllIntervals();
  }
}

// ===============================================================
// BUTTON HANDLER
// ===============================================================
void handleButtonEvent(ButtonEvent btn) {
  if (btn == BTN_NONE) return;

  if (btn == BTN_MENU_EVENT) {
    resetSystem(); menuState = MENU_MAIN; menuIndex = 0;
    lcd.clear(); lcd.print("Reset Done"); delay(400); lcd.clear();
    return;
  }

  if (btn == BTN_ENTER_EVENT) {
    if (menuState == MENU_MAIN) {
      if (menuIndex == 0) { menuState = MENU_MEASURE; startMeasurement(); lcd.clear(); return; }
      if (menuIndex == 1) { menuState = MENU_VIEW;    lcd.clear(); return; }
      if (menuIndex == 2) { resetSystem(); lcd.clear(); }
    } else if (menuState == MENU_VIEW) {
      menuState = MENU_MAIN; lcd.clear(); return;
    }
    menuIndex++;
    if (menuIndex > 2) menuIndex = 0;
    lastMenuIndex = -1;
    return;
  }

  if (btn == BTN_GATE_EVENT) {
    if (!systemActive) {
      numGates++;
      if (numGates > MAX_GATES) numGates = MIN_GATES;
      lastNumGates = -1;
    }
    return;
  }
}

// ===============================================================
void displayMenu() {
  if (menuIndex != lastMenuIndex) {
    lcd.setCursor(0,0);
    if (menuIndex == 0) lcd.print("> Start Meas   ");
    if (menuIndex == 1) lcd.print("> View Data    ");
    if (menuIndex == 2) lcd.print("> Reset        ");
    lastMenuIndex = menuIndex;
  }
  if (numGates != lastNumGates) {
    lcd.setCursor(0,1);
    lcd.print("Gates:");
    lcd.print(numGates);
    // แสดง IP สั้นๆ
    String ip = (WiFi.status()==WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    lcd.print(" ");
    lcd.print(ip.substring(ip.lastIndexOf('.')+1));  // แสดงเฉพาะ octet สุดท้าย
    lcd.print("   ");
    lastNumGates = numGates;
  }
}

// ===============================================================
void startMeasurement() {
  portENTER_CRITICAL(&mux);
  currentGate = 0; measurementDone = false; lastTriggerTime = 0;
  portEXIT_CRITICAL(&mux);
  systemActive = true; measurementComplete = false; armed = false;
  digitalWrite(LED_PIN, HIGH);
  lcd.clear(); lcd.print("Waiting clear...");
}

// ===============================================================
void runMeasurement() {
  uint8_t localGate;
  portENTER_CRITICAL(&mux); localGate = currentGate; portEXIT_CRITICAL(&mux);
  if (localGate != lastGateCount) {
    lcd.setCursor(0,0); lcd.print("Measuring...   ");
    lcd.setCursor(0,1); lcd.print(localGate); lcd.print("/"); lcd.print(numGates); lcd.print("   ");
    lastGateCount = localGate;
  }
}

// ===============================================================
void showAllIntervals() {
  uint32_t localTimes[MAX_GATES];
  uint8_t  localGate;
  portENTER_CRITICAL(&mux);
  localGate = currentGate;
  for (int i = 0; i < localGate; i++) localTimes[i] = gateTimes[i];
  portEXIT_CRITICAL(&mux);

  measurementComplete = true;
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n=== INTERVALS (ms) ===");
  for (int i = 1; i < localGate; i++) {
    float dt = (localTimes[i] - localTimes[i-1]) / 1000.0;
    Serial.print("Gate "); Serial.print(i); Serial.print(" -> Gate "); Serial.print(i+1);
    Serial.print(": "); Serial.print(dt, 3); Serial.println(" ms");
  }

  lcd.clear(); lcd.print("Done! "); lcd.print(localGate); lcd.print(" gates");
  delay(800);
  menuState = MENU_VIEW;
}

// ===============================================================
void displayIntervals() {
  uint32_t localTimes[MAX_GATES];
  uint8_t  localGate;
  portENTER_CRITICAL(&mux);
  localGate = currentGate;
  for (int i = 0; i < localGate; i++) localTimes[i] = gateTimes[i];
  portEXIT_CRITICAL(&mux);

  if (localGate < 2) {
    lcd.setCursor(0,0); lcd.print("No Data Yet     ");
    lcd.setCursor(0,1); lcd.print("                ");
    return;
  }
  if (millis() - lastDisplayUpdate < 1200) return;
  lastDisplayUpdate = millis();
  if (displayGate >= localGate - 1) displayGate = 0;

  float dt = (localTimes[displayGate+1] - localTimes[displayGate]) / 1000.0;
  lcd.setCursor(0,0); lcd.print("G"); lcd.print(displayGate+1); lcd.print("->G"); lcd.print(displayGate+2); lcd.print("        ");
  lcd.setCursor(0,1); lcd.print(dt, 3); lcd.print(" ms     ");
  displayGate++;
}

// ===============================================================
// SERIAL COMMAND HANDLER
// ===============================================================
void handleSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuffer.trim(); serialBuffer.toUpperCase();
      if (serialBuffer.length() > 0) processSerialCommand(serialBuffer);
      serialBuffer = "";
    } else {
      if (serialBuffer.length() < 32) serialBuffer += c;
    }
  }
}

void processSerialCommand(String cmd) {
  if (cmd == "HELP") {
    Serial.println("=== Photogate Serial Commands ===");
    Serial.println("  DUMP     - Export intervals as CSV");
    Serial.println("  DUMP_RAW - Export raw timestamps (us)");
    Serial.println("  STATUS   - Show system status");
    Serial.println("  RESET    - Reset system");
    Serial.println("  GATES N  - Set gate count (2-20)");
    Serial.println("  SEP N    - Set gate separation in meters");
    Serial.println("  HELP     - Show this help");
    return;
  }
  if (cmd == "STATUS") {
    uint8_t localGate;
    portENTER_CRITICAL(&mux); localGate = currentGate; portEXIT_CRITICAL(&mux);
    Serial.println("=== STATUS ===");
    Serial.print("System active : "); Serial.println(systemActive        ? "YES" : "NO");
    Serial.print("Armed         : "); Serial.println(armed               ? "YES" : "NO");
    Serial.print("Meas complete : "); Serial.println(measurementComplete ? "YES" : "NO");
    Serial.print("Num gates set : "); Serial.println(numGates);
    Serial.print("Gates captured: "); Serial.println(localGate);
    Serial.print("Gate sep (m)  : "); Serial.println(gateSeparation_m, 4);
    String ip = (WiFi.status()==WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    Serial.print("Web IP        : "); Serial.println(ip);
    return;
  }
  if (cmd == "RESET") {
    resetSystem(); menuState = MENU_MAIN; menuIndex = 0; lcd.clear();
    Serial.println("OK: System reset");
    return;
  }
  if (cmd == "DUMP") {
    uint32_t localTimes[MAX_GATES]; uint8_t localGate;
    portENTER_CRITICAL(&mux); localGate = currentGate;
    for (int i = 0; i < localGate; i++) localTimes[i] = gateTimes[i];
    portEXIT_CRITICAL(&mux);
    if (localGate < 2) { Serial.println("ERROR: No data"); return; }
    Serial.println("=== DUMP CSV ===");
    Serial.println("interval_index,from_gate,to_gate,interval_ms");
    for (int i = 1; i < localGate; i++) {
      float dt = (localTimes[i] - localTimes[i-1]) / 1000.0;
      Serial.print(i); Serial.print(","); Serial.print(i); Serial.print(",");
      Serial.print(i+1); Serial.print(","); Serial.println(dt, 3);
    }
    Serial.println("=== END DUMP ===");
    return;
  }
  if (cmd == "DUMP_RAW") {
    uint32_t localTimes[MAX_GATES]; uint8_t localGate;
    portENTER_CRITICAL(&mux); localGate = currentGate;
    for (int i = 0; i < localGate; i++) localTimes[i] = gateTimes[i];
    portEXIT_CRITICAL(&mux);
    if (localGate < 1) { Serial.println("ERROR: No data"); return; }
    Serial.println("=== DUMP_RAW ===");
    Serial.println("gate_index,timestamp_us,delta_us");
    for (int i = 0; i < localGate; i++) {
      Serial.print(i+1); Serial.print(","); Serial.print(localTimes[i]); Serial.print(",");
      Serial.println(i == 0 ? 0 : localTimes[i] - localTimes[i-1]);
    }
    Serial.println("=== END DUMP_RAW ===");
    return;
  }
  if (cmd.startsWith("GATES ")) {
    int n = cmd.substring(6).toInt();
    if (n < MIN_GATES || n > MAX_GATES) { Serial.println("ERROR: Gates 2-20"); return; }
    if (systemActive) { Serial.println("ERROR: Measuring"); return; }
    numGates = n; lastNumGates = -1;
    Serial.print("OK: Gates = "); Serial.println(n);
    return;
  }
  if (cmd.startsWith("SEP ")) {
    float v = cmd.substring(4).toFloat();
    if (v <= 0) { Serial.println("ERROR: sep must be > 0"); return; }
    gateSeparation_m = v;
    Serial.print("OK: Separation = "); Serial.print(v, 4); Serial.println(" m");
    return;
  }
  Serial.print("ERROR: Unknown '"); Serial.print(cmd); Serial.println("' — HELP");
}

// ===============================================================
void resetSystem() {
  portENTER_CRITICAL(&mux);
  currentGate = 0; measurementDone = false; lastTriggerTime = 0;
  portEXIT_CRITICAL(&mux);
  measurementComplete = false; systemActive = false;
  displayGate = 0; armed = false; measurementStartTime = 0;
  for (int i = 0; i < MAX_GATES; i++) gateTimes[i] = 0;
  digitalWrite(LED_PIN, LOW);
  lastMenuIndex = -1; lastNumGates = -1; lastGateCount = -1;
}
