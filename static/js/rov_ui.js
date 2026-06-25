// ─────────────────────────────────────────────────────────────
// GLOBAL STATE (must be first — before Socket.IO connect)
// ─────────────────────────────────────────────────────────────
let _tel    = {};
let _status = {};
let _currentLog  = 'arm';
const _logs = { thrust: [], arm: [], onboard_stab: [], onboard_arm: [], onboard_cam: [], colmap: [], crabs: [] };
let _onboardPollTimer = null;
const _onboardProgressSeen = new Set();
let _telemetryRecording = false;
let _lastBatteryToast = '';
let _missionPollTimer = null;
let socket = null;

function socketEmit(event, data) {
  if (socket && socket.connected) socket.emit(event, data);
}

// ─────────────────────────────────────────────────────────────
// SOCKET.IO
// ─────────────────────────────────────────────────────────────
if (typeof io !== 'undefined') {
  socket = io({ transports: ['websocket', 'polling'] });

  socket.on('connect', () => {
    socketEmit('request_status');
  });

  socket.on('telemetry', (data) => {
    _tel = data;
    if (data.link_health) _status.link_health = data.link_health;
    updateTelemetry();
    updateCtrlCmdsFromTelemetry();
  });

  socket.on('status', (data) => {
    _status = data;
    updateStatus();
  });

  socket.on('process_log', ({ name, line }) => {
    const logs = _logs[name] || (_logs[name] = []);
    logs.push(line);
    if (logs.length > 300) logs.splice(0, logs.length - 300);
    if (name === _currentLog) appendLogLine(line);
  });

  socket.on('onboard_progress', (entry) => {
    handleOnboardProgress(entry);
  });
} else {
  console.error('Socket.IO client failed to load — live updates use HTTP polling fallback');
}

function handleOnboardProgress({ step, status, msg, time }) {
  const key = `${time || ''}|${step}|${status}|${msg}`;
  if (_onboardProgressSeen.has(key)) return;
  _onboardProgressSeen.add(key);

  const logEl = document.getElementById('onboard-progress-log');
  if (logEl) {
    logEl.style.display = 'block';
    const icons = { starting: '⟳', wait: '…', done: '✓', error: '✕', complete: '★' };
    const cls   = { starting: 'info', wait: 'wait', done: 'ok', error: 'err', complete: 'ok' };
    const div = document.createElement('div');
    div.className = 'pl-step ' + (cls[status] || 'info');
    div.textContent = `${icons[status] || '?'} [${step}] ${msg || status}`;
    logEl.appendChild(div);
    logEl.scrollTop = logEl.scrollHeight;
  }

  // Update status dots immediately from progress events
  if (step === 'mavproxy') {
    if (status === 'done') setDot('dot-mavproxy', true);
    if (status === 'error') setDotError('dot-mavproxy');
  }
  if (step === 'stabilization') {
    if (status === 'done') setDot('dot-stab', true);
    if (status === 'error') setDotError('dot-stab');
  }
  if (step === 'arm_ctrl') {
    if (status === 'done') setDot('dot-arm', true);
    if (status === 'error') setDotError('dot-arm');
  }
  if (step === 'camera') {
    if (status === 'done') setDot('dot-cam', true);
    if (status === 'error') setDotError('dot-cam');
  }

  const summary = document.getElementById('onboard-summary');
  if (summary && msg) {
    if (status === 'done' || status === 'complete') {
      summary.style.color = 'var(--green)';
      summary.textContent = msg;
    } else if (status === 'error') {
      summary.style.color = 'var(--red)';
      summary.textContent = msg;
    } else if (status === 'starting' || status === 'wait') {
      summary.style.color = 'var(--amber)';
      summary.textContent = msg;
    }
  }

  if (step === 'complete') {
    toast(msg, status === 'done' ? 'ok' : 'err');
    const btn = document.getElementById('btn-start-onboard');
    if (btn) btn.disabled = false;
    stopOnboardPoll();
  }
}

function startOnboardPoll() {
  stopOnboardPoll();

  async function pollOnce() {
    try {
      const r = await fetch('/api/onboard/progress');
      const d = await r.json();
      if (d.events && d.events.length) {
        const lastSeen = _onboardPollTimer ? (_onboardPollTimer._lastCount || 0) : 0;
        for (let i = lastSeen; i < d.events.length; i++) {
          handleOnboardProgress(d.events[i]);
        }
        if (_onboardPollTimer) _onboardPollTimer._lastCount = d.events.length;
      }
      if (d.onboard_mavproxy) setDot('dot-mavproxy', true);
      if (d.onboard_stab)     setDot('dot-stab', true);
      if (d.onboard_arm)      setDot('dot-arm', true);
      if (d.onboard_cam)      setDot('dot-cam', true);
      if (!d.starting) stopOnboardPoll();
    } catch (_) {}
  }

  _onboardPollTimer = setInterval(pollOnce, 800);
  _onboardPollTimer._lastCount = 0;
  pollOnce();
}

function stopOnboardPoll() {
  if (_onboardPollTimer) {
    clearInterval(_onboardPollTimer);
    _onboardPollTimer = null;
  }
}

// ─────────────────────────────────────────────────────────────
// CONFIG HELPERS
// ─────────────────────────────────────────────────────────────
function getCfg() {
  const keys = ['pi_ip','pi_user','pi_password','pi_ssh_port','pi_rov_path',
                 'serial_port','forward_camera_url','arm_camera_url',
                 'camera0_device','camera1_device',
                 'thrust_udp_port','telemetry_port','arm_udp_port',
                 'mosfet_control_port','colmap_command','crabs_command',
                 'battery_warn_v','battery_crit_v',
                 'mavproxy_bin','mavproxy_serial','mavproxy_baud',
                 'mavproxy_out1','mavproxy_out2'];
  const obj = {};
  keys.forEach(k => {
    const el = document.getElementById('cfg-' + k);
    if (el) obj[k] = el.value;
  });
  return obj;
}

async function saveConfig() {
  await fetch('/api/config', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(getCfg()),
  });
}

async function loadConfig() {
  try {
    const r = await fetch('/api/config');
    const cfg = await r.json();
    Object.entries(cfg).forEach(([k, v]) => {
      const el = document.getElementById('cfg-' + k);
      if (el && v !== null && v !== undefined) el.value = v;
    });
    if (cfg.telemetry_port) CTRL_CFG.TELEMETRY_PORT = parseInt(cfg.telemetry_port, 10) || 5006;
  } catch (_) {}
}

// ─────────────────────────────────────────────────────────────
// SSH
// ─────────────────────────────────────────────────────────────
async function sshConnect() {
  await saveConfig();
  const cfg   = getCfg();
  const badge = document.getElementById('ssh-status');
  badge.textContent = 'Connecting…';
  badge.className   = 'ssh-badge';

  const r = await fetch('/api/ssh/connect', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(cfg),
  });
  const d = await r.json();
  badge.textContent = d.ok ? '✓ Connected' : '✕ ' + d.msg;
  badge.className   = 'ssh-badge ' + (d.ok ? 'ok' : 'err');
  toast(d.ok ? 'SSH connected to ' + cfg.pi_ip : 'SSH failed: ' + d.msg, d.ok ? 'ok' : 'err');
}

// ─────────────────────────────────────────────────────────────
// LAUNCH ACTIONS
// ─────────────────────────────────────────────────────────────
async function startOnboard() {
  await saveConfig();
  const logEl = document.getElementById('onboard-progress-log');
  const summary = document.getElementById('onboard-summary');
  if (logEl) { logEl.innerHTML = ''; logEl.style.display = 'block'; }
  if (summary) { summary.textContent = 'Starting onboard programs…'; summary.style.color = 'var(--amber)'; }
  _onboardProgressSeen.clear();
  const btn = document.getElementById('btn-start-onboard');
  if (btn) btn.disabled = true;

  startOnboardPoll();

  const r = await fetch('/api/onboard/start', { method: 'POST' });
  const d = await r.json();
  if (!d.ok) {
    if (logEl) {
      const div = document.createElement('div');
      div.className = 'pl-step err';
      div.textContent = '✕ ' + d.msg;
      logEl.appendChild(div);
    }
    if (summary) { summary.textContent = '✕ ' + d.msg; summary.style.color = 'var(--red)'; }
    if (btn) btn.disabled = false;
    stopOnboardPoll();
    toast('Onboard start failed: ' + d.msg, 'err');
  } else if (d.in_progress) {
    toast(d.msg, 'warn');
  }
}

async function stopOnboard() {
  await fetch('/api/onboard/stop', { method: 'POST' });
  toast('Onboard programs stopped');
  const logEl = document.getElementById('onboard-progress-log');
  if (logEl) { logEl.innerHTML = ''; logEl.style.display = 'none'; }
}

async function startTopside() {
  await saveConfig();
  const msg = document.getElementById('topside-msg');
  msg.textContent = 'Starting arm_sender.py…';
  const r = await fetch('/api/topside/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(getCfg()),
  });
  const d = await r.json();
  const armRes = d.results && d.results.arm_sender;
  msg.textContent = armRes ? `arm_sender: ${armRes.ok ? '✓ '+armRes.msg : '✕ '+armRes.msg}` : JSON.stringify(d);
  toast(armRes && armRes.ok ? 'Arm sender started' : 'Arm sender failed', armRes && armRes.ok ? 'ok' : 'err');
}

async function stopTopside() {
  await fetch('/api/topside/stop', { method: 'POST' });
  document.getElementById('topside-msg').textContent = 'Stopped';
  toast('Topside programs stopped');
}

// ─────────────────────────────────────────────────────────────
// CONTROL ACTIONS
// ─────────────────────────────────────────────────────────────
let _mosfetOn = false;

async function toggleMosfet() {
  _mosfetOn = !_mosfetOn;
  await fetch('/api/mosfet', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ state: _mosfetOn }),
  });
  updateMosfetUI(_mosfetOn);
  toast('MOSFET ' + (_mosfetOn ? 'ON' : 'OFF'), _mosfetOn ? 'ok' : '');
}

function updateMosfetUI(on) {
  const toggle = document.getElementById('mosfet-toggle');
  const label  = document.getElementById('mosfet-label');
  if (on) { toggle.classList.add('on');    label.textContent = 'MOSFET ON'; }
  else    { toggle.classList.remove('on'); label.textContent = 'MOSFET OFF'; }
}

let _currentMode = 'disarmed';

async function setMode(mode) {
  const r = await fetch('/api/mode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode }),
  });
  const d = await r.json();
  if (d.ok) {
    _currentMode = mode;
    updateModeUI(mode);

    // Auto-configure ctrl flags based on mode
    if (mode === 'disarmed') {
      // Clear all flags (they're ignored anyway, but keep UI clean)
      _ctrlState.stabilize  = false;
      _ctrlState.depth_hold = false;
      _ctrlState.yaw_hold   = false;
    } else if (mode === 'stabilize') {
      // Stabilize mode enables stabilization automatically
      _ctrlState.stabilize = true;
    } else if (mode === 'armed') {
      // Armed: manual control, stabilize off by default
      _ctrlState.stabilize = false;
    }
    updateFlagUI();
  }
  const modeNames = { disarmed:'DISARMED', armed:'DRIVE/ARMED', stabilize:'STABILIZE' };
  toast('Mode: ' + (modeNames[mode] || mode.toUpperCase()), mode === 'disarmed' ? '' : 'ok');
}

function updateModeUI(mode) {
  ['disarmed','armed','stabilize'].forEach(m => {
    const btn = document.getElementById('mode-' + m);
    if (btn) btn.className = 'mode-btn' + (mode === m ? ' active-' + m : '');
  });

  const banner = document.getElementById('disarmed-banner');
  if (banner) banner.classList.toggle('hidden', mode !== 'disarmed');

  const pillMode = document.getElementById('pill-mode');
  const modeColors = { disarmed:'err', armed:'warn', stabilize:'ok' };
  if (pillMode) {
    pillMode.innerHTML = `<span class="status-dot"></span> ${(mode||'--').toUpperCase()}`;
    pillMode.className = 'status-pill ' + (modeColors[mode] || '');
  }
}

async function startColmap() {
  const r = await fetch('/api/colmap', { method: 'POST' });
  const d = await r.json();
  toast(d.ok ? '▶ COLMAP started' : 'COLMAP failed: ' + d.msg, d.ok ? 'ok' : 'err');
  refreshMissionStatus();
}

async function startCrabs() {
  const r = await fetch('/api/crabs', { method: 'POST' });
  const d = await r.json();
  toast(d.ok ? '🦀 Crabs started' : 'Crabs failed: ' + d.msg, d.ok ? 'ok' : 'err');
  refreshMissionStatus();
}

async function sendArmPreset(name) {
  const r = await fetch(`/api/arm_preset/${encodeURIComponent(name)}`, { method: 'POST' });
  const d = await r.json();
  const label = (_armPresets[name] && _armPresets[name].label) || name;
  toast(d.ok ? `Arm preset: ${label}` : d.msg, d.ok ? 'ok' : 'err');
}

// ─────────────────────────────────────────────────────────────
// ARM PRESETS
// ─────────────────────────────────────────────────────────────
const ARM_JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'Claw'];
let _armPresets = {};
let _armLastPwm = null;
let _armPresetEditing = null;

async function loadArmPresets() {
  try {
    const r = await fetch('/api/arm_presets');
    const d = await r.json();
    if (!d.ok) return;
    _armPresets = d.presets || {};
    if (d.current) _armLastPwm = d.current;
    renderArmPresetButtons();
    renderArmPresetList();
    updateArmCurrentDisplay();
  } catch (_) {}
}

function renderArmPresetButtons() {
  const wrap = document.getElementById('arm-preset-btns');
  if (!wrap) return;
  const names = Object.keys(_armPresets);
  if (!names.length) {
    wrap.innerHTML = '<span style="font-size:.68rem;color:var(--dim)">No presets</span>';
    return;
  }
  wrap.innerHTML = names.map(name => {
    const label = (_armPresets[name] && _armPresets[name].label) || name;
    return `<button type="button" class="action-btn" onclick="sendArmPreset('${name}')" title="${name}">${escapeHtml(label)}</button>`;
  }).join('');
}

function buildArmPwmGrid() {
  const grid = document.getElementById('arm-preset-pwm-grid');
  if (!grid || grid.childElementCount) return;
  grid.innerHTML = ARM_JOINT_NAMES.map((jn, i) => `
    <div class="field">
      <label>${jn} µs</label>
      <input id="arm-pwm-${i}" type="number" min="500" max="2500" step="1" value="1500"/>
    </div>
  `).join('');
}

function getArmFormValues() {
  const pwm = [];
  for (let i = 0; i < 7; i++) {
    const el = document.getElementById('arm-pwm-' + i);
    pwm.push(el ? parseInt(el.value, 10) || 1500 : 1500);
  }
  const j6El = document.getElementById('arm-preset-j6');
  const j6 = j6El ? parseFloat(j6El.value) : 0;
  const nameEl = document.getElementById('arm-preset-name');
  const labelEl = document.getElementById('arm-preset-label');
  return {
    name: nameEl ? nameEl.value.trim() : '',
    label: labelEl ? labelEl.value.trim() : '',
    pwm,
    j6_angle: Number.isFinite(j6) ? j6 : 0,
  };
}

function setArmFormValues(preset, name) {
  _armPresetEditing = name || null;
  const title = document.getElementById('arm-preset-form-title');
  if (title) title.textContent = name ? `Edit: ${name}` : 'Add preset';
  const nameEl = document.getElementById('arm-preset-name');
  const labelEl = document.getElementById('arm-preset-label');
  const j6El = document.getElementById('arm-preset-j6');
  if (nameEl) {
    nameEl.value = name || '';
    nameEl.disabled = !!name;
  }
  if (labelEl) labelEl.value = (preset && preset.label) || '';
  if (j6El) j6El.value = (preset && preset.j6_angle != null) ? preset.j6_angle : 0;
  for (let i = 0; i < 7; i++) {
    const el = document.getElementById('arm-pwm-' + i);
    if (el) el.value = (preset && preset.pwm && preset.pwm[i] != null) ? preset.pwm[i] : 1500;
  }
}

function resetArmPresetForm() {
  _armPresetEditing = null;
  setArmFormValues({ pwm: [1500, 1500, 1500, 1500, 1500, 1500, 1500], j6_angle: 0, label: '' }, '');
  const nameEl = document.getElementById('arm-preset-name');
  if (nameEl) nameEl.disabled = false;
  const title = document.getElementById('arm-preset-form-title');
  if (title) title.textContent = 'Add preset';
}

function updateArmCurrentDisplay() {
  const el = document.getElementById('arm-preset-current');
  if (!el) return;
  if (!_armLastPwm || !_armLastPwm.pwm) {
    el.textContent = 'Current: -- (start arm_sender and move arm to record)';
    return;
  }
  el.textContent = `Current: ${_armLastPwm.pwm.join(', ')} | J6 ${(_armLastPwm.j6_angle ?? 0).toFixed(1)}°`;
}

function recordArmCurrentIntoForm() {
  if (!_armLastPwm || !_armLastPwm.pwm) {
    toast('No current arm PWM — start arm_sender and move the arm first', 'err');
    return;
  }
  const keepName = document.getElementById('arm-preset-name')?.value || '';
  const keepLabel = document.getElementById('arm-preset-label')?.value || '';
  setArmFormValues({
    label: keepLabel,
    pwm: _armLastPwm.pwm.slice(),
    j6_angle: _armLastPwm.j6_angle ?? 0,
  }, _armPresetEditing || keepName);
  toast('Recorded current arm position', 'ok');
}

async function saveArmPresetFromForm() {
  const v = getArmFormValues();
  if (!v.name) {
    toast('Enter a preset name', 'err');
    return;
  }
  const r = await fetch('/api/arm_presets', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(v),
  });
  const d = await r.json();
  if (d.ok) {
    toast(`Saved preset: ${d.name}`, 'ok');
    await loadArmPresets();
    setArmFormValues(d.preset, d.name);
  } else {
    toast(d.msg || 'Save failed', 'err');
  }
}

async function deleteArmPreset(name) {
  if (!confirm(`Delete preset "${name}"?`)) return;
  const r = await fetch(`/api/arm_presets/${encodeURIComponent(name)}`, { method: 'DELETE' });
  const d = await r.json();
  if (d.ok) {
    toast(`Deleted: ${name}`, 'ok');
    if (_armPresetEditing === name) resetArmPresetForm();
    await loadArmPresets();
  } else {
    toast(d.msg || 'Delete failed', 'err');
  }
}

function renderArmPresetList() {
  const list = document.getElementById('arm-preset-list');
  if (!list) return;
  const names = Object.keys(_armPresets);
  if (!names.length) {
    list.innerHTML = '<div style="font-size:.78rem;color:var(--dim)">No presets yet.</div>';
    return;
  }
  list.innerHTML = names.map(name => {
    const p = _armPresets[name];
    const meta = (p.pwm || []).join(', ') + ` | J6 ${(p.j6_angle ?? 0).toFixed(1)}°`;
    const label = escapeHtml(p.label || name);
    return `<div class="arm-preset-row">
      <div class="arm-preset-row-name">${label}
        <div class="arm-preset-row-meta">${escapeHtml(meta)}</div>
      </div>
      <div class="arm-preset-row-actions">
        <button type="button" onclick="sendArmPreset('${name}')">Go</button>
        <button type="button" onclick="editArmPreset('${name}')">Edit</button>
        <button type="button" class="danger" onclick="deleteArmPreset('${name}')">Del</button>
      </div>
    </div>`;
  }).join('');
}

function editArmPreset(name) {
  const p = _armPresets[name];
  if (!p) return;
  setArmFormValues(p, name);
}

function showArmPresets() {
  buildArmPwmGrid();
  resetArmPresetForm();
  loadArmPresets();
  document.getElementById('arm-presets-modal').style.display = 'flex';
}

function hideArmPresets() {
  document.getElementById('arm-presets-modal').style.display = 'none';
}

function hideArmPresetsOutside(e) {
  if (e.target === document.getElementById('arm-presets-modal')) hideArmPresets();
}

async function toggleTelemetryRecord() {
  const action = _telemetryRecording ? 'stop' : 'start';
  const r = await fetch('/api/telemetry_record', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action }),
  });
  const d = await r.json();
  if (d.ok) {
    _telemetryRecording = !!d.recording;
    updateRecordButton();
    toast(
      d.recording ? `Recording → ${d.file}` : `Recording stopped (${d.file || 'none'})`,
      d.recording ? 'ok' : ''
    );
  }
}

function updateRecordButton() {
  const btn = document.getElementById('btn-record');
  if (!btn) return;
  btn.textContent = _telemetryRecording ? '⏹ REC' : '⏺ REC';
  btn.classList.toggle('recording', _telemetryRecording);
}

async function takeSnapshot(camNum) {
  const camNames = { 1: 'forward', 2: 'arm' };
  try {
    const r = await fetch(`/camera/${camNum}/snapshot?t=${Date.now()}`);
    if (!r.ok) throw new Error(await r.text());
    const blob = await r.blob();
    const img = await createImageBitmap(blob);

    const canvas = document.createElement('canvas');
    canvas.width = img.width;
    canvas.height = img.height;
    const ctx = canvas.getContext('2d');
    ctx.drawImage(img, 0, 0);

    const t = _tel;
    const stamp = new Date().toISOString().replace(/[:.]/g, '-');
    const lines = [
      `DreadYachet ROV — ${camNames[camNum] || camNum} cam`,
      `Depth: ${fmtNum(t.depth_m, 2, 'm')}  Yaw: ${fmtNum(t.yaw_deg, 1, '°')}`,
      `Roll: ${fmtNum(t.roll_deg, 1, '°')}  Pitch: ${fmtNum(t.pitch_deg, 1, '°')}`,
      `Battery: ${fmtNum(t.battery_voltage_v, 2, 'V')}  ${fmtNum(t.battery_current_a, 1, 'A')}`,
      `State: ${t.rx_state || '--'}  ${stamp}`,
    ];
    ctx.fillStyle = 'rgba(0,0,0,0.55)';
    ctx.fillRect(0, canvas.height - 28 * lines.length - 8, canvas.width, 28 * lines.length + 8);
    ctx.fillStyle = '#00e08a';
    ctx.font = '16px monospace';
    lines.forEach((line, i) => {
      ctx.fillText(line, 12, canvas.height - 28 * (lines.length - i) + 4);
    });

    canvas.toBlob((png) => {
      const a = document.createElement('a');
      a.href = URL.createObjectURL(png);
      a.download = `rov_${camNames[camNum] || camNum}_${stamp}.png`;
      a.click();
      URL.revokeObjectURL(a.href);
      toast(`Snapshot saved (${camNames[camNum]})`, 'ok');
    }, 'image/png');
  } catch (e) {
    toast('Snapshot failed: ' + e.message, 'err');
  }
}

async function refreshMissionStatus() {
  try {
    const r = await fetch('/api/mission_status');
    const d = await r.json();
    const colEl = document.getElementById('mission-colmap');
    const crbEl = document.getElementById('mission-crabs');
    if (colEl && d.colmap) {
      const st = d.colmap.running ? 'RUNNING' : 'idle';
      colEl.textContent = `COLMAP: ${st}`;
      colEl.className = 'mission-item ' + (d.colmap.running ? 'running' : 'stopped');
      colEl.title = d.colmap.last_line || '';
    }
    if (crbEl && d.crabs) {
      const st = d.crabs.running ? 'RUNNING' : 'idle';
      crbEl.textContent = `CRABS: ${st}`;
      crbEl.className = 'mission-item ' + (d.crabs.running ? 'running' : 'stopped');
      crbEl.title = d.crabs.last_line || '';
    }
  } catch (_) {}
}

function startMissionPoll() {
  stopMissionPoll();
  refreshMissionStatus();
  _missionPollTimer = setInterval(refreshMissionStatus, 5000);
}

function stopMissionPoll() {
  if (_missionPollTimer) { clearInterval(_missionPollTimer); _missionPollTimer = null; }
}

// ─────────────────────────────────────────────────────────────
// CONTROL FLAG TOGGLES (clickable from control bar + S/D/Y keys)
// ─────────────────────────────────────────────────────────────
function toggleStabilize() {
  if (_currentMode === 'disarmed') { toast('Switch to ARMED or STABILIZE mode first', 'warn'); return; }
  _ctrlState.stabilize = !_ctrlState.stabilize;
  updateFlagUI();
  toast(`Stabilization: ${_ctrlState.stabilize ? 'ON' : 'OFF'}`, _ctrlState.stabilize ? 'ok' : '');
}

function toggleDepthHold() {
  if (_currentMode === 'disarmed') { toast('Switch to ARMED or STABILIZE mode first', 'warn'); return; }
  _ctrlState.depth_hold = !_ctrlState.depth_hold;
  updateFlagUI();
  toast(`Depth Hold: ${_ctrlState.depth_hold ? 'ON' : 'OFF'}`, _ctrlState.depth_hold ? 'ok' : '');
}

function toggleYawHold() {
  if (_currentMode === 'disarmed') { toast('Switch to ARMED or STABILIZE mode first', 'warn'); return; }
  _ctrlState.yaw_hold = !_ctrlState.yaw_hold;
  updateFlagUI();
  toast(`Yaw Hold: ${_ctrlState.yaw_hold ? 'ON' : 'OFF'}`, _ctrlState.yaw_hold ? 'ok' : '');
}

function calibrateIMU() {
  const sending = _currentMode === 'armed' || _currentMode === 'stabilize';
  sendCtrlPacket({
    seq: _ctrlState.seq++,
    time: Date.now() / 1000,
    forward:  sending ? _localCmds.forward  : 0,
    lateral:  sending ? _localCmds.lateral  : 0,
    yaw:      sending ? _localCmds.yaw      : 0,
    vertical: sending ? _localCmds.vertical : 0,
    stabilize:   sending ? _ctrlState.stabilize  : false,
    depth_hold:  sending ? _ctrlState.depth_hold : false,
    yaw_hold:    sending ? _ctrlState.yaw_hold   : false,
    gain_percent: _ctrlState.gain_percent,
    telemetry_port: CTRL_CFG.TELEMETRY_PORT,
    calibrate_imu: true,
  });
  toast('IMU zero sent — current pitch/roll set as targets', 'ok');
}

function updateFlagUI() {
  const stabBtn  = document.getElementById('flag-stab');
  const depthBtn = document.getElementById('flag-depth');
  const yawBtn   = document.getElementById('flag-yaw');

  if (stabBtn) {
    stabBtn.textContent = `STAB: ${_ctrlState.stabilize ? 'ON' : 'OFF'}`;
    stabBtn.className   = 'ctrl-flag' + (_ctrlState.stabilize ? ' active-stab' : '');
  }
  if (depthBtn) {
    depthBtn.textContent = `DEPTH: ${_ctrlState.depth_hold ? 'ON' : 'OFF'}`;
    depthBtn.className   = 'ctrl-flag' + (_ctrlState.depth_hold ? ' active-depth' : '');
  }
  if (yawBtn) {
    yawBtn.textContent = `YAW: ${_ctrlState.yaw_hold ? 'ON' : 'OFF'}`;
    yawBtn.className   = 'ctrl-flag' + (_ctrlState.yaw_hold ? ' active-yaw' : '');
  }
}

// ─────────────────────────────────────────────────────────────
// STATUS UPDATES
// ─────────────────────────────────────────────────────────────
function updateStatus() {
  const s = _status;

  setDot('dot-mavproxy', s.onboard_mavproxy);
  setDot('dot-stab',     s.onboard_stab);
  setDot('dot-arm',      s.onboard_arm);
  setDot('dot-cam',      s.onboard_cam);
  setDot('dot-armlocal', s.arm_running);
  updateOpenControlButton(s);

  // Telemetry listener status on launch screen
  const telemDot = document.getElementById('dot-telem-launch');
  const telemLbl = document.getElementById('telem-launch-label');
  if (telemDot) {
    if (s.telemetry_listener_ok) {
      telemDot.className = 'dot running';
      if (telemLbl) telemLbl.textContent = 'Telemetry listener active (UDP 5006)';
    } else {
      telemDot.className = 'dot error';
      if (telemLbl) telemLbl.textContent = 'Telemetry listener FAILED — port 5006 in use?';
    }
  }

  // Replay onboard progress if we connected mid-start
  if (s.onboard_progress && s.onboard_progress.length) {
    const last = s.onboard_progress[s.onboard_progress.length - 1];
    const summary = document.getElementById('onboard-summary');
    if (summary && last.msg) {
      summary.textContent = last.msg;
      summary.style.color = last.status === 'error' ? 'var(--red)'
        : (last.status === 'done' || last.step === 'complete') ? 'var(--green)' : 'var(--amber)';
    }
  }

  const badge = document.getElementById('ssh-status');
  if (s.ssh_connected) {
    badge.textContent = '✓ Connected';
    badge.className   = 'ssh-badge ok';
  } else if (s.ssh_error) {
    badge.textContent = '✕ ' + s.ssh_error.substring(0, 40);
    badge.className   = 'ssh-badge err';
  }

  const pillConn = document.getElementById('pill-connection');
  if (pillConn) {
    pillConn.innerHTML  = `<span class="status-dot"></span> SSH: ${s.ssh_connected ? 'ONLINE' : 'OFFLINE'}`;
    pillConn.className  = 'status-pill ' + (s.ssh_connected ? 'ok' : 'err');
  }

  if (s.mode) { _currentMode = s.mode; updateModeUI(s.mode); }
  if (typeof s.mosfet_on !== 'undefined') { _mosfetOn = s.mosfet_on; updateMosfetUI(s.mosfet_on); }

  const ap = document.getElementById('tel-arm-proc');
  if (ap) {
    ap.textContent = s.arm_running ? 'RUN' : 'STOP';
    ap.className   = 'tc-val ' + (s.arm_running ? 'good' : 'bad');
  }

  if (typeof s.telemetry_recording !== 'undefined') {
    _telemetryRecording = s.telemetry_recording;
    updateRecordButton();
  }

  if (s.arm_last_pwm) {
    _armLastPwm = s.arm_last_pwm;
    updateArmCurrentDisplay();
  }

  updateLinkHealthPill(s.link_health);
  updatePreDiveChecklist(s);
}

function updateLinkHealthPill(link) {
  const pill = document.getElementById('pill-link');
  if (!pill) return;
  if (!link) {
    pill.innerHTML = '<span class="status-dot"></span> LINK: --';
    pill.className = 'status-pill';
    return;
  }
  const label = link.level === 'ok' ? 'OK' : link.level.toUpperCase();
  pill.innerHTML = `<span class="status-dot"></span> LINK: ${label}`;
  pill.className = 'status-pill ' + (link.level === 'ok' ? 'ok' : link.level === 'warn' ? 'warn' : 'err');
  pill.title = `${link.detail || ''} | tel ${link.telemetry_age_sec}s @ ${link.telemetry_rate_hz}Hz | ctrl ${link.ctrl_age_sec}s`;
}

function updatePreDiveChecklist(s) {
  const gp = _getActiveGamepad();
  const items = [
    { id: 'chk-ssh', ok: s.ssh_connected },
    { id: 'chk-mavproxy', ok: s.onboard_mavproxy },
    { id: 'chk-stab', ok: s.onboard_stab },
    { id: 'chk-arm-onboard', ok: s.onboard_arm },
    { id: 'chk-cam', ok: s.onboard_cam },
    { id: 'chk-arm-sender', ok: s.arm_running },
    { id: 'chk-telem', ok: s.telemetry_listener_ok },
    { id: 'chk-gamepad', ok: !!gp },
  ];
  items.forEach(({ id, ok }) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.className = ok ? 'ok' : '';
    const icon = el.querySelector('.chk-icon');
    if (icon) icon.textContent = ok ? '✓' : '○';
  });
}

function _getActiveGamepad() {
  const gps = navigator.getGamepads ? navigator.getGamepads() : [];
  for (let i = 0; i < gps.length; i++) {
    if (gps[i]) return gps[i];
  }
  return null;
}

function setDot(id, running) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = 'dot ' + (running ? 'running' : '');
}

function setDotError(id) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = 'dot error';
}

// ─────────────────────────────────────────────────────────────
// TELEMETRY UPDATES
// ─────────────────────────────────────────────────────────────
function fmtNum(v, dec=2, unit='') {
  if (v === null || v === undefined) return '--';
  return parseFloat(v).toFixed(dec) + unit;
}

function updateTelemetry() {
  const t = _tel;

  // Telemetry pill
  const pillTel = document.getElementById('pill-telemetry');
  const telOk   = t.rx_state === 'OK';
  if (pillTel) {
    let label = t.rx_state || '--';
    if (label === 'NO_TELEMETRY') label = 'NO TELEM';
    pillTel.innerHTML  = `<span class="status-dot"></span> ${label}`;
    pillTel.title = (t.rx_state === 'NO_TELEMETRY')
      ? 'Start onboard programs, then telemetry appears on UDP 5006'
      : '';
    pillTel.className  = 'status-pill ' + (
      telOk ? 'ok' : (t.rx_state === 'NO_TELEMETRY' ? 'err' : 'warn')
    );
  }

  // Gain
  document.getElementById('tb-gain').textContent = t.gain_percent ?? _ctrlState.gain_percent;

  // Forward camera telemetry overlay
  setText('cam-depth',  fmtNum(t.depth_m, 2));
  setText('cam-hold-d', fmtNum(t.hold_depth_m, 2));
  setText('cam-yaw',    fmtNum(t.yaw_deg, 1));
  setText('cam-roll',   fmtNum(t.roll_deg, 1));
  setText('cam-pitch',  fmtNum(t.pitch_deg, 1));
  const stabEl = document.getElementById('cam-stab');
  if (stabEl) {
    stabEl.textContent = t.stabilize ? 'ON' : 'OFF';
    stabEl.className = t.stabilize ? 'val-hi' : '';
  }
  const battCam = document.getElementById('cam-batt');
  if (battCam) {
    battCam.textContent = t.battery_voltage_v != null ? fmtNum(t.battery_voltage_v, 2) : '--';
    battCam.className = _batteryClass(t.battery_voltage_v);
  }

  _checkBatteryAlerts(t.battery_voltage_v);

  // Telemetry bar
  const state = t.rx_state || '--';
  const stateEl = document.getElementById('tel-state');
  if (stateEl) {
    stateEl.textContent = state;
    stateEl.className   = 'tc-val ' + (state === 'OK' ? 'good' : state === 'NO_TELEMETRY' ? '' : 'warn');
  }

  setText('tel-depth',  fmtNum(t.depth_m, 2, 'm'));
  setText('tel-hold-d', fmtNum(t.hold_depth_m, 2, 'm'));
  setText('tel-yaw',    fmtNum(t.yaw_deg, 1, '°'));
  setText('tel-hold-y', fmtNum(t.hold_yaw_deg, 1, '°'));
  setText('tel-roll',   fmtNum(t.roll_deg, 1, '°'));
  setText('tel-pitch',  fmtNum(t.pitch_deg, 1, '°'));
  setText('tel-hgrp',   fmtNum(t.h_group, 2));
  setText('tel-vgrp',   fmtNum(t.v_group, 2));
  setText('tel-press',  t.pressure_hpa ? fmtNum(t.pressure_hpa, 0, 'hPa') : '--');
  setText('tel-temp',   t.temperature_c ? fmtNum(t.temperature_c, 1, '°C') : '--');

  const battV = document.getElementById('tel-batt-v');
  if (battV) {
    battV.textContent = t.battery_voltage_v != null ? fmtNum(t.battery_voltage_v, 2, 'V') : '--';
    battV.className = 'tc-val ' + _batteryTcClass(t.battery_voltage_v);
  }
  setText('tel-batt-a',   t.battery_current_a != null ? fmtNum(t.battery_current_a, 1, 'A') : '--');
  setText('tel-batt-pct', t.battery_remaining_pct != null ? fmtNum(t.battery_remaining_pct, 0, '%') : '--');

  const linkEl = document.getElementById('tel-link');
  const lh = t.link_health || _status.link_health;
  if (linkEl && lh) {
    linkEl.textContent = lh.level === 'ok' ? 'OK' : lh.level.toUpperCase();
    linkEl.className = 'tc-val ' + (lh.level === 'ok' ? 'good' : lh.level === 'warn' ? 'warn' : 'bad');
    setText('tel-rate', fmtNum(lh.telemetry_rate_hz, 1, 'Hz'));
    updateLinkHealthPill(lh);
  }

  const dhEl = document.getElementById('tel-dh');
  if (dhEl) {
    dhEl.textContent = t.depth_hold_active ? 'HOLD' : (t.depth_hold_request ? 'WAIT' : 'OFF');
    dhEl.className   = 'tc-val ' + (t.depth_hold_active ? 'good' : t.depth_hold_request ? 'warn' : '');
  }
  const yhEl = document.getElementById('tel-yh');
  if (yhEl) {
    yhEl.textContent = t.yaw_hold_active ? 'HOLD' : (t.yaw_hold_request ? 'WAIT' : 'OFF');
    yhEl.className   = 'tc-val ' + (t.yaw_hold_active ? 'good' : t.yaw_hold_request ? 'warn' : '');
  }
}

function updateCtrlCmdsFromTelemetry() {
  // CMD values come from our local gamepad loop, not from telemetry
  // but update gain if telemetry has a different value (e.g. from future sync)
}

function setText(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function _batteryClass(voltage) {
  if (voltage == null) return '';
  const warn = parseFloat(document.getElementById('cfg-battery_warn_v')?.value) || 12.0;
  const crit = parseFloat(document.getElementById('cfg-battery_crit_v')?.value) || 11.0;
  if (voltage <= crit) return 'val-warn';
  if (voltage <= warn) return 'val-warn';
  return 'val-hi';
}

function _batteryTcClass(voltage) {
  if (voltage == null) return '';
  const warn = parseFloat(document.getElementById('cfg-battery_warn_v')?.value) || 12.0;
  const crit = parseFloat(document.getElementById('cfg-battery_crit_v')?.value) || 11.0;
  if (voltage <= crit) return 'bad';
  if (voltage <= warn) return 'warn';
  return 'good';
}

function _checkBatteryAlerts(voltage) {
  if (voltage == null) return;
  const warn = parseFloat(document.getElementById('cfg-battery_warn_v')?.value) || 12.0;
  const crit = parseFloat(document.getElementById('cfg-battery_crit_v')?.value) || 11.0;
  let level = '';
  if (voltage <= crit) level = 'crit';
  else if (voltage <= warn) level = 'warn';
  if (!level || level === _lastBatteryToast) return;
  _lastBatteryToast = level;
  if (level === 'crit') toast(`LOW BATTERY ${voltage.toFixed(2)}V — critical`, 'err');
  else toast(`Battery low ${voltage.toFixed(2)}V`, 'warn');
  if (voltage > warn) _lastBatteryToast = '';
}

// ─────────────────────────────────────────────────────────────
// GAMEPAD CONTROL — matches thrust_sender.py exactly
// ─────────────────────────────────────────────────────────────

// Config (mirrors thrust_sender.py constants)
const CTRL_CFG = {
  AXIS_LEFT_X:      0,
  AXIS_LEFT_Y:      1,
  AXIS_RIGHT_X:     3,
  AXIS_RIGHT_Y:     4,
  SIGN_YAW:         1.0,
  SIGN_VERTICAL:   -1.0,
  SIGN_LATERAL:     1.0,
  SIGN_FORWARD:    -1.0,
  DEADZONE:         0.05,
  GAIN_MIN:         10,
  GAIN_MAX:         100,
  GAIN_STEP:        10,
  GAIN_DEFAULT:     100,
  BUTTON_STABILIZE: 9,
  COMBINED_LIMIT:   1.50,
  SEND_HZ:          50,
  TELEMETRY_PORT:   5006,
};

let _ctrlLayout = 'original';  // 'original' or 'swapped'

let _ctrlState = {
  stabilize:    false,
  depth_hold:   false,
  yaw_hold:     false,
  gain_percent: CTRL_CFG.GAIN_DEFAULT,
  seq:          0,
};

// Last locally-computed commands (for HUD display even without telemetry)
let _localCmds = { forward: 0, lateral: 0, yaw: 0, vertical: 0 };

// Button / hat edge detection state
let _btnPrev       = [];
let _dpadUpPrev    = false;
let _dpadDownPrev  = false;
let _lastSendMs    = 0;

function _applyDeadzone(x, dz) {
  return Math.abs(x) < dz ? 0.0 : x;
}

function _clamp(x, lo, hi) {
  return Math.max(lo, Math.min(hi, x));
}

function _applyCombinedLimit(f, l, y, v, limit) {
  const h     = Math.max(Math.abs(f), Math.abs(l), Math.abs(y));
  const vAbs  = Math.abs(v);
  const total = h + vAbs;
  if (total <= limit || total <= 1e-6) {
    return { f, l, y, v, scale: 1.0, h, vAbs, total };
  }
  const scale = limit / total;
  return {
    f: f * scale, l: l * scale, y: y * scale, v: v * scale,
    scale, h: h * scale, vAbs: vAbs * scale, total: total * scale,
  };
}

function _adjustGain(delta) {
  _ctrlState.gain_percent = Math.round(
    _clamp(_ctrlState.gain_percent + delta, CTRL_CFG.GAIN_MIN, CTRL_CFG.GAIN_MAX)
  );
  const el = document.getElementById('tb-gain');
  if (el) el.textContent = _ctrlState.gain_percent;
  toast(`Gain: ${_ctrlState.gain_percent}%`);
}

// ── Keyboard handlers (global, match thrust_sender.py keybinds) ──
const _keyDown = {};
document.addEventListener('keydown', (e) => {
  if (_keyDown[e.key]) return;  // key held
  _keyDown[e.key] = true;

  const onCtrl = document.getElementById('control').classList.contains('active');
  if (!onCtrl) return;

  switch (e.key) {
    case 's': case 'S':
      toggleStabilize();
      break;
    case 'd': case 'D':
      toggleDepthHold();
      break;
    case 'y': case 'Y':
      toggleYawHold();
      break;
    case 'c': case 'C':
      calibrateIMU();
      break;
    case 'ArrowUp':
      e.preventDefault();
      _adjustGain(CTRL_CFG.GAIN_STEP);
      break;
    case 'ArrowDown':
      e.preventDefault();
      _adjustGain(-CTRL_CFG.GAIN_STEP);
      break;
    case 'Escape':
      // Emergency stop
      setMode('disarmed');
      toast('EMERGENCY STOP — DISARMED', 'err');
      break;
  }
});
document.addEventListener('keyup', (e) => { _keyDown[e.key] = false; });

// ── Gamepad connection events ──
let _gamepadActivated = false;

function activateGamepad() {
  _gamepadActivated = true;
  const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
  let found = false;
  for (let i = 0; i < gamepads.length; i++) {
    if (gamepads[i]) { found = true; break; }
  }
  if (found) {
    toast('Gamepad detected!', 'ok');
  } else {
    toast('Press ANY button on your gamepad now…', 'warn');
  }
  _updateGamepadPill();
}

window.addEventListener('gamepadconnected', (e) => {
  _gamepadActivated = true;
  toast(`Gamepad connected: ${e.gamepad.id.substring(0, 40)}`, 'ok');
  _updateGamepadPill();
});
window.addEventListener('gamepaddisconnected', () => {
  toast('Gamepad disconnected!', 'err');
  _updateGamepadPill();
});

function _findGamepad() {
  const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (let i = 0; i < gamepads.length; i++) {
    if (gamepads[i]) return gamepads[i];
  }
  return null;
}

function _updateGamepadPill() {
  const gp = _findGamepad();

  const pill = document.getElementById('pill-gamepad');
  if (pill) {
    if (gp) {
      const name = gp.id.length > 22 ? gp.id.substring(0, 22) + '…' : gp.id;
      pill.innerHTML = `<span class="status-dot"></span> GP: ${name}`;
      pill.className = 'status-pill ok';
    } else {
      pill.innerHTML = `<span class="status-dot"></span> GP: NONE`;
      pill.className = 'status-pill err';
    }
  }

  const launchDot = document.getElementById('dot-gamepad-launch');
  const launchLbl = document.getElementById('gamepad-launch-label');
  if (launchDot) {
    launchDot.className = 'dot ' + (gp ? 'running' : '');
  }
  if (launchLbl) {
    launchLbl.textContent = gp
      ? `Gamepad: ${gp.id.substring(0, 36)}`
      : 'Gamepad — click Activate, then press any button';
  }

  const actBtn = document.getElementById('btn-activate-gp');
  if (actBtn && gp) actBtn.textContent = '✓ Gamepad Active';
}

let _lastHttpCtrlSend = 0;

function sendCtrlPacket(packet) {
  socketEmit('ctrl_packet', packet);
  const nowMs = performance.now();
  const useHttp = !socket || !socket.connected || (nowMs - _lastHttpCtrlSend > 250);
  if (useHttp) {
    _lastHttpCtrlSend = nowMs;
    fetch('/api/ctrl', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(packet),
    }).catch(() => {});
  }
}

// ── Main gamepad + control send loop ──
function _gamepadControlLoop() {
  requestAnimationFrame(_gamepadControlLoop);

  const nowMs = performance.now();
  const intervalMs = 1000 / CTRL_CFG.SEND_HZ;
  if (nowMs - _lastSendMs < intervalMs) return;
  _lastSendMs = nowMs;

  const onCtrl = document.getElementById('control')?.classList.contains('active');
  const gp = _findGamepad();
  _updateGamepadPill();

  // Any button press wakes gamepad in most browsers
  if (gp) {
    for (let b = 0; b < gp.buttons.length; b++) {
      if (gp.buttons[b].pressed) { _gamepadActivated = true; break; }
    }
  }

  let forward = 0, lateral = 0, yaw = 0, vertical = 0;

  if (gp && _gamepadActivated) {
    const leftX  = gp.axes[CTRL_CFG.AXIS_LEFT_X]  || 0;
    const leftY  = gp.axes[CTRL_CFG.AXIS_LEFT_Y]  || 0;
    const rightX = gp.axes[CTRL_CFG.AXIS_RIGHT_X] || 0;
    const rightY = gp.axes[CTRL_CFG.AXIS_RIGHT_Y] || 0;

    let axisYaw, axisLateral;
    if (_ctrlLayout === 'original') {
      axisYaw     = leftX;
      axisLateral = rightX;
    } else {
      axisYaw     = rightX;
      axisLateral = leftX;
    }

    const yawRaw  = _clamp(_applyDeadzone(CTRL_CFG.SIGN_YAW      * axisYaw,    CTRL_CFG.DEADZONE), -1, 1);
    const vertRaw = _clamp(_applyDeadzone(CTRL_CFG.SIGN_VERTICAL  * leftY,      CTRL_CFG.DEADZONE), -1, 1);
    const latRaw  = _clamp(_applyDeadzone(CTRL_CFG.SIGN_LATERAL   * axisLateral,CTRL_CFG.DEADZONE), -1, 1);
    const fwdRaw  = _clamp(_applyDeadzone(CTRL_CFG.SIGN_FORWARD   * rightY,     CTRL_CFG.DEADZONE), -1, 1);

    const gain = _ctrlState.gain_percent / 100.0;
    const r = _applyCombinedLimit(
      fwdRaw * gain, latRaw * gain, yawRaw * gain, vertRaw * gain,
      CTRL_CFG.COMBINED_LIMIT
    );
    forward  = r.f;
    lateral  = r.l;
    yaw      = r.y;
    vertical = r.v;

    const numBtns = gp.buttons.length;
    if (_btnPrev.length !== numBtns) _btnPrev = new Array(numBtns).fill(false);

    for (let b = 0; b < numBtns; b++) {
      const pressed = gp.buttons[b].pressed;
      if (pressed && !_btnPrev[b]) {
        console.log(`[Gamepad] Button ${b} pressed`);
        if (onCtrl && b === CTRL_CFG.BUTTON_STABILIZE) toggleStabilize();
      }
      _btnPrev[b] = pressed;
    }

    const dpadAxisY = (gp.axes.length > 7) ? (gp.axes[7] || 0) : 0;
    const dpadUp   = (gp.buttons[12] && gp.buttons[12].pressed) || dpadAxisY < -0.5;
    const dpadDown = (gp.buttons[13] && gp.buttons[13].pressed) || dpadAxisY > 0.5;
    if (onCtrl) {
      if (dpadUp   && !_dpadUpPrev)   _adjustGain( CTRL_CFG.GAIN_STEP);
      if (dpadDown && !_dpadDownPrev) _adjustGain(-CTRL_CFG.GAIN_STEP);
    }
    _dpadUpPrev   = dpadUp;
    _dpadDownPrev = dpadDown;
  }

  // Show raw stick demand when disarmed (HUD preview) but send zeros to Pi
  const sending = onCtrl && (_currentMode === 'armed' || _currentMode === 'stabilize');
  let sendForward = forward, sendLateral = lateral, sendYaw = yaw, sendVertical = vertical;
  if (!sending) { sendForward = 0; sendLateral = 0; sendYaw = 0; sendVertical = 0; }

  _localCmds = { forward, lateral, yaw, vertical };
  _tel.cmd_forward  = forward;
  _tel.cmd_lateral  = lateral;
  _tel.cmd_yaw      = yaw;
  _tel.cmd_vertical = vertical;

  const fmtCmd = v => (Math.abs(v) < 0.005 ? '0.00' : (v >= 0 ? '+' : '') + v.toFixed(2));
  setText('tel-cmd-f', fmtCmd(sendForward));
  setText('tel-cmd-l', fmtCmd(sendLateral));
  setText('tel-cmd-y', fmtCmd(sendYaw));
  setText('tel-cmd-v', fmtCmd(sendVertical));

  const packet = {
    seq:         _ctrlState.seq++,
    time:        nowMs / 1000,
    forward:     sendForward,
    lateral:     sendLateral,
    yaw:         sendYaw,
    vertical:    sendVertical,
    stabilize:   sending ? _ctrlState.stabilize  : false,
    depth_hold:  sending ? _ctrlState.depth_hold : false,
    yaw_hold:    sending ? _ctrlState.yaw_hold   : false,
    gain_percent: _ctrlState.gain_percent,
    telemetry_port: CTRL_CFG.TELEMETRY_PORT,
  };
  sendCtrlPacket(packet);
}

// ── Axis layout setter ──
function setLayout(layout) {
  _ctrlLayout = layout;

  const orig = document.getElementById('layout-btn-original');
  const swap = document.getElementById('layout-btn-swapped');
  if (orig) { orig.className = 'layout-btn' + (layout === 'original' ? ' active' : ''); }
  if (swap) { swap.className = 'layout-btn' + (layout === 'swapped'  ? ' active' : ''); }

  const disp = document.getElementById('kb-layout-display');
  if (disp) disp.textContent = layout === 'original' ? 'ORIGINAL' : 'SWAPPED';

  const lxl = document.getElementById('kb-axis-left-x-label');
  const lxd = document.getElementById('kb-axis-left-x-desc');
  const rxl = document.getElementById('kb-axis-right-x-label');
  const rxd = document.getElementById('kb-axis-right-x-desc');

  if (layout === 'original') {
    if (lxl) lxl.textContent = 'Left Stick X';
    if (lxd) lxd.textContent = 'Yaw (rotate) — left = turn left, right = turn right';
    if (rxl) rxl.textContent = 'Right Stick X';
    if (rxd) rxd.textContent = 'Lateral strafe — left = strafe left, right = strafe right';
  } else {
    if (lxl) lxl.textContent = 'Left Stick X';
    if (lxd) lxd.textContent = 'Lateral strafe — left = strafe left, right = strafe right';
    if (rxl) rxl.textContent = 'Right Stick X';
    if (rxd) rxd.textContent = 'Yaw (rotate) — left = turn left, right = turn right';
  }

  toast(`Axis layout: ${layout.toUpperCase()}`);
}

// ─────────────────────────────────────────────────────────────
// ATTITUDE + DIRECTION HUD CANVAS
// ─────────────────────────────────────────────────────────────
function telAngle(v) {
  if (v === null || v === undefined || v === '') return null;
  const n = parseFloat(v);
  return Number.isFinite(n) ? n : null;
}

function normYaw360(deg) {
  return ((deg % 360) + 360) % 360;
}

function drawLegibleText(ctx, text, x, y, color, align) {
  if (align) ctx.textAlign = align;
  ctx.fillStyle = 'rgba(0,0,0,0.85)';
  ctx.fillText(text, x + 1, y + 1);
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
}

function drawAttitudeOverlay(ctx, W, H, rollDeg, pitchDeg, yawDeg, live) {
  const cx = W * 0.5;
  const cy = H * 0.5;
  const pitchScale = H / 75;
  const rollRad = rollDeg * Math.PI / 180;
  const pitchOff = pitchDeg * pitchScale;
  const span = W * 1.4;

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(rollRad);
  ctx.translate(0, pitchOff);

  ctx.fillStyle = 'rgba(0, 95, 135, 0.06)';
  ctx.fillRect(-span, -H * 2, span * 2, H * 2);
  ctx.fillStyle = 'rgba(0, 12, 28, 0.12)';
  ctx.fillRect(-span, 0, span * 2, H * 2);

  for (let p = -40; p <= 40; p += 10) {
    const y = -p * pitchScale;
    const major = p === 0;
    const half = major ? span * 0.8 : (Math.abs(p) % 20 === 0 ? span * 0.55 : span * 0.28);

    ctx.strokeStyle = major
      ? (live ? 'rgba(0,212,255,0.30)' : 'rgba(255,179,32,0.28)')
      : 'rgba(0,212,255,0.28)';
    ctx.lineWidth = major ? 1 : 1;
    ctx.beginPath();
    ctx.moveTo(-half, y);
    ctx.lineTo(half, y);
    ctx.stroke();

    if (!major && Math.abs(p) <= 30) {
      const lbl = String(Math.abs(p));
      ctx.font = `bold ${Math.max(9, H * 0.026)}px monospace`;
      ctx.textAlign = 'left';
      drawLegibleText(ctx, lbl, half + 6, y + 3, 'rgba(0,224,138,0.95)');
      ctx.textAlign = 'right';
      drawLegibleText(ctx, lbl, -half - 6, y + 3, 'rgba(0,224,138,0.95)');
    }
  }

  ctx.restore();

  const wing = Math.min(W, H) * 0.11;
  ctx.strokeStyle = live ? 'rgba(0,212,255,0.95)' : 'rgba(255,179,32,0.65)';
  ctx.fillStyle = ctx.strokeStyle;
  ctx.lineWidth = 2.5;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(cx - wing, cy);
  ctx.lineTo(cx - wing * 0.18, cy);
  ctx.moveTo(cx + wing * 0.18, cy);
  ctx.lineTo(cx + wing, cy);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(cx, cy, 3, 0, Math.PI * 2);
  ctx.fill();

  const arcR = Math.min(W, H) * 0.24;
  const arcCy = cy - arcR * 0.55;
  ctx.strokeStyle = 'rgba(0,212,255,0.45)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(cx, arcCy, arcR, Math.PI * 1.12, Math.PI * 1.88);
  ctx.stroke();

  for (let deg = -60; deg <= 60; deg += 10) {
    const a = -Math.PI / 2 + deg * Math.PI / 180;
    const tick = deg === 0 || Math.abs(deg) === 30 || Math.abs(deg) === 60 ? 11 : 6;
    const x1 = cx + arcR * Math.cos(a);
    const y1 = arcCy + arcR * Math.sin(a);
    const x2 = cx + (arcR + tick) * Math.cos(a);
    const y2 = arcCy + (arcR + tick) * Math.sin(a);
    ctx.strokeStyle = deg === 0 ? 'rgba(0,224,138,0.85)' : 'rgba(0,212,255,0.45)';
    ctx.lineWidth = deg === 0 ? 2 : 1;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }

  ctx.save();
  ctx.translate(cx, arcCy);
  ctx.rotate(rollRad);
  ctx.fillStyle = live ? 'rgba(0,224,138,0.95)' : 'rgba(255,179,32,0.85)';
  ctx.beginPath();
  ctx.moveTo(0, -arcR + 2);
  ctx.lineTo(-8, -arcR + 18);
  ctx.lineTo(8, -arcR + 18);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  drawYawTape(ctx, W, H, yawDeg, live);
}

function drawYawTape(ctx, W, H, yawDeg, live) {
  const tapeY = H - 34;
  const tapeH = 20;
  const cx = W * 0.5;
  const pxPerDeg = Math.max(2.2, W / 180);

  ctx.fillStyle = 'rgba(0,0,0,0.5)';
  ctx.fillRect(0, tapeY - 6, W, tapeH + 14);
  ctx.strokeStyle = 'rgba(0,212,255,0.2)';
  ctx.lineWidth = 1;
  ctx.strokeRect(0.5, tapeY - 6.5, W - 1, tapeH + 13);

  const centerYaw = live ? normYaw360(yawDeg) : 0;
  const visible = W / pxPerDeg;
  const start = centerYaw - visible * 0.5;

  for (let d = Math.floor(start / 5) * 5; d <= start + visible + 5; d += 5) {
    const normD = normYaw360(d);
    const x = cx + (d - centerYaw) * pxPerDeg;
    if (x < -24 || x > W + 24) continue;

    const major = normD % 30 === 0;
    ctx.strokeStyle = major ? 'rgba(0,212,255,0.75)' : 'rgba(0,212,255,0.28)';
    ctx.lineWidth = major ? 1.5 : 1;
    ctx.beginPath();
    ctx.moveTo(x, tapeY);
    ctx.lineTo(x, tapeY + (major ? 12 : 7));
    ctx.stroke();

    if (major) {
      ctx.font = `bold ${Math.max(10, H * 0.024)}px monospace`;
      ctx.textAlign = 'center';
      drawLegibleText(ctx, normD.toFixed(0).padStart(3, '0'), x, tapeY + tapeH + 2,
                      'rgba(255,255,255,0.98)', 'center');
    }
  }

  ctx.fillStyle = live ? 'rgba(0,224,138,0.95)' : 'rgba(255,179,32,0.85)';
  ctx.beginPath();
  ctx.moveTo(cx, tapeY - 4);
  ctx.lineTo(cx - 7, tapeY + 5);
  ctx.lineTo(cx + 7, tapeY + 5);
  ctx.closePath();
  ctx.fill();

  ctx.font = `bold ${Math.max(9, H * 0.02)}px sans-serif`;
  drawLegibleText(ctx, 'HDG', cx, tapeY - 8, 'rgba(0,224,138,0.95)', 'center');

  if (!live) {
    ctx.font = `bold ${Math.max(9, H * 0.02)}px sans-serif`;
    drawLegibleText(ctx, 'NO IMU', cx, H - 6, 'rgba(255,179,32,0.95)', 'center');
  }
}

function drawCommandMiniHUD(ctx, W, H, t) {
  const size = Math.min(W, H) * 0.17;
  const cx = size + 14;
  const cy = H - size - 52;
  const R = size * 0.72;

  const fwd  = t.cmd_forward  || 0;
  const lat  = t.cmd_lateral  || 0;
  const yaw  = t.cmd_yaw      || 0;
  const vert = t.cmd_vertical || 0;

  ctx.fillStyle = 'rgba(0,0,0,0.45)';
  ctx.beginPath();
  ctx.arc(cx, cy, R + 10, 0, Math.PI * 2);
  ctx.fill();

  ctx.beginPath();
  ctx.arc(cx, cy, R, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(0,212,255,0.22)';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  ctx.strokeStyle = 'rgba(0,212,255,0.15)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
  ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
  ctx.stroke();

  if (Math.abs(fwd) > 0.02) {
    drawArrow(ctx, cx, cy, cx, cy - fwd * R * 0.82, '#00e08a', Math.abs(fwd));
  }
  if (Math.abs(lat) > 0.02) {
    drawArrow(ctx, cx, cy, cx + lat * R * 0.82, cy, '#ffb320', Math.abs(lat));
  }
  if (Math.abs(yaw) > 0.02) {
    const startA = -Math.PI / 2;
    const sweepA = yaw * Math.PI * 0.85;
    ctx.beginPath();
    ctx.arc(cx, cy, R * 0.86, startA, startA + sweepA, yaw < 0);
    ctx.strokeStyle = `rgba(255,100,100,${Math.min(1, Math.abs(yaw) * 0.7 + 0.3)})`;
    ctx.lineWidth = 2.5;
    ctx.stroke();
  }

  if (Math.abs(vert) > 0.02) {
    const bx = cx + R + 8;
    const bh = R * 0.65;
    ctx.beginPath();
    ctx.roundRect(bx - 3, cy - bh, 6, bh * 2, 3);
    ctx.fillStyle = 'rgba(0,212,255,0.08)';
    ctx.fill();
    const barH = Math.abs(vert) * bh;
    const barY = vert > 0 ? cy - barH : cy;
    ctx.beginPath();
    ctx.roundRect(bx - 3, barY, 6, barH, 2);
    ctx.fillStyle = vert > 0
      ? `rgba(0,224,138,${Math.abs(vert) * 0.7 + 0.3})`
      : `rgba(255,61,90,${Math.abs(vert) * 0.7 + 0.3})`;
    ctx.fill();
  }

  ctx.fillStyle = 'rgba(0,212,255,0.55)';
  ctx.font = `bold ${Math.max(7, R * 0.22)}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.fillText('CMD', cx, cy - R - 6);
}

function drawHUD(canvasId, t) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const wrap = canvas.parentElement;
  canvas.width  = wrap.clientWidth;
  canvas.height = wrap.clientHeight;

  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const W = canvas.width;
  const H = canvas.height;

  const roll  = telAngle(t.roll_deg);
  const pitch = telAngle(t.pitch_deg);
  const yaw   = telAngle(t.yaw_deg);
  const live  = roll !== null && pitch !== null && yaw !== null;

  drawAttitudeOverlay(
    ctx, W, H,
    live ? roll : 0,
    live ? pitch : 0,
    live ? yaw : 0,
    live
  );
  drawCommandMiniHUD(ctx, W, H, t);

  const modeLabels = { disarmed:'DISARMED', armed:'ARMED', stabilize:'STABILIZE' };
  const modeColors = { disarmed:'rgba(255,61,90,0.85)', armed:'rgba(255,179,32,0.85)',
                       stabilize:'rgba(0,224,138,0.85)' };
  const mode = _currentMode || 'disarmed';
  ctx.font = `bold ${Math.max(9, H * 0.022)}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.fillStyle = modeColors[mode] || 'rgba(200,200,200,0.6)';
  ctx.fillText(modeLabels[mode] || mode.toUpperCase(), W * 0.5, H - 42);
}

function drawArrow(ctx, x1, y1, x2, y2, color, opacity) {
  const dx = x2 - x1, dy = y2 - y1;
  const len = Math.sqrt(dx * dx + dy * dy);
  if (len < 4) return;
  const angle   = Math.atan2(dy, dx);
  const headLen = Math.min(14, len * 0.35);

  ctx.globalAlpha = Math.min(1, opacity * 0.7 + 0.3);
  ctx.strokeStyle = color;
  ctx.fillStyle   = color;
  ctx.lineWidth   = 2.5;

  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - headLen * Math.cos(angle - Math.PI / 6),
             y2 - headLen * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(x2 - headLen * Math.cos(angle + Math.PI / 6),
             y2 - headLen * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();
  ctx.globalAlpha = 1;
}

// ─────────────────────────────────────────────────────────────
// CAMERA VIEW MODES
// ─────────────────────────────────────────────────────────────
let _cameraView = 'split';

const _CAMERA_VIEW_LABELS = {
  split:   'Side by side',
  pip:     'Forward + arm (PiP)',
  forward: 'Forward only',
  arm:     'Arm only',
};

function setCameraView(mode) {
  if (!_CAMERA_VIEW_LABELS[mode]) return;
  _cameraView = mode;

  const el = document.getElementById('cameras');
  if (el) el.className = 'cameras view-' + mode;

  document.querySelectorAll('.cam-view-btn').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.view === mode);
  });

  resizeHUDs();
  requestAnimationFrame(() => {
    const control = document.getElementById('control');
    if (!control || !control.classList.contains('active')) return;
    const showForward = mode !== 'arm';
    const showArm = mode !== 'forward';
    const cam1 = document.getElementById('cam1');
    const cam2 = document.getElementById('cam2');
    if (showForward && cam1 && cam1.style.display !== 'block') {
      setupCamera('cam1', 'no-sig-1', 1);
    }
    if (showArm && cam2 && cam2.style.display !== 'block') {
      setTimeout(() => setupCamera('cam2', 'no-sig-2', 2), 300);
    }
  });
}

// ─────────────────────────────────────────────────────────────
// CAMERA SETUP
// ─────────────────────────────────────────────────────────────
// UI slot → config URL field (matches rov_ui.py _CAMERA_UI_URL_KEY)
const _CAMERA_CFG_KEY = { 1: 'forward_camera_url', 2: 'arm_camera_url' };
const _cameraState = {};
let _camResizeBound = false;

function cameraDirectUrl(camNum) {
  const cfgKey = _CAMERA_CFG_KEY[camNum];
  const el = cfgKey ? document.getElementById('cfg-' + cfgKey) : null;
  return el ? el.value.trim() : '';
}

function stopCamera(camNum) {
  const st = _cameraState[camNum];
  if (!st) return;
  st.active = false;
  if (st.retryT) { clearTimeout(st.retryT); st.retryT = null; }
  if (st.watchdogT) { clearTimeout(st.watchdogT); st.watchdogT = null; }
  if (st.img) {
    st.img.onload = null;
    st.img.onerror = null;
    st.img.removeAttribute('src');
    st.img.style.display = 'none';
  }
  delete _cameraState[camNum];
}

function stopAllCameras() {
  stopCamera(1);
  stopCamera(2);
}

function setupCamera(imgId, noSigId, camNum) {
  stopCamera(camNum);

  const img = document.getElementById(imgId);
  const noSig = document.getElementById(noSigId);
  if (!img || !noSig) return;

  const st = {
    img, noSig, camNum,
    retryT: null,
    watchdogT: null,
    failCount: 0,
    active: true,
    lastLoad: 0,
  };
  _cameraState[camNum] = st;

  function showNoSignal() {
    noSig.style.display = 'flex';
    img.style.display = 'none';
    const nsText = noSig.querySelector('.ns-text');
    if (nsText) {
      const upstream = cameraDirectUrl(camNum) || 'not configured';
      const camNames = { 1: 'Forward Camera', 2: 'Arm Camera' };
      const camLabel = camNames[camNum] || `Cam ${camNum}`;
      nsText.textContent = `No Signal — ${camLabel} (upstream ${upstream})`;
    }
  }

  function onSuccess() {
    if (!st.active) return;
    st.failCount = 0;
    if (st.watchdogT) { clearTimeout(st.watchdogT); st.watchdogT = null; }
    noSig.style.display = 'none';
    img.style.display = 'block';
  }

  function scheduleRetry() {
    if (!st.active || st.retryT) return;
    const delay = Math.min(8000, 800 + st.failCount * 700);
    st.retryT = setTimeout(() => {
      st.retryT = null;
      loadStream(true);
    }, delay);
  }

  function onFail() {
    if (!st.active) return;
    st.failCount++;
    showNoSignal();
    scheduleRetry();
  }

  function loadStream(forceReconnect) {
    if (!st.active) return;
    const control = document.getElementById('control');
    if (!control || !control.classList.contains('active')) return;

    st.lastLoad = Date.now();
    if (forceReconnect) {
      img.onload = null;
      img.onerror = null;
      img.removeAttribute('src');
    }

    const attach = () => {
      if (!st.active) return;
      img.onload = onSuccess;
      img.onerror = onFail;
      img.src = `/camera/${camNum}?t=${Date.now()}`;
      if (st.watchdogT) clearTimeout(st.watchdogT);
      st.watchdogT = setTimeout(() => {
        st.watchdogT = null;
        if (!st.active || img.style.display === 'block') return;
        loadStream(true);
      }, 12000);
    };

    if (forceReconnect && img.src) {
      requestAnimationFrame(attach);
    } else {
      attach();
    }
  }

  showNoSignal();
  loadStream(false);
}

function startAllCameras() {
  stopAllCameras();
  setupCamera('cam1', 'no-sig-1', 1);
  setTimeout(() => setupCamera('cam2', 'no-sig-2', 2), 450);
}

function bindCameraResize() {
  if (_camResizeBound) return;
  window.addEventListener('resize', resizeHUDs);
  _camResizeBound = true;
}

function unbindCameraResize() {
  if (!_camResizeBound) return;
  window.removeEventListener('resize', resizeHUDs);
  _camResizeBound = false;
}

// ─────────────────────────────────────────────────────────────
// VIEW SWITCHING
// ─────────────────────────────────────────────────────────────
const _ONBOARD_SYSTEMS = [
  { key: 'onboard_mavproxy', label: 'mavproxy (UDP bridge)' },
  { key: 'onboard_stab',     label: 'stabilization.py' },
  { key: 'onboard_arm',      label: 'new_ar.py' },
  { key: 'onboard_cam',      label: 'camera_stream.py' },
];

function getOfflineOnboardSystems(status) {
  const s = status || _status || {};
  return _ONBOARD_SYSTEMS.filter(sys => !s[sys.key]).map(sys => sys.label);
}

function updateOpenControlButton(status) {
  const btn = document.getElementById('btn-open-control');
  if (!btn) return;
  const ready = getOfflineOnboardSystems(status).length === 0;
  btn.classList.toggle('btn-success', ready);
  btn.classList.toggle('not-ready', !ready);
}

function showOfflineConfirm(offline) {
  const list = document.getElementById('offline-systems-list');
  if (list) {
    list.innerHTML = offline.map(label => `<li>${label}</li>`).join('');
  }
  document.getElementById('offline-confirm-modal').style.display = 'flex';
}

function hideOfflineConfirm() {
  document.getElementById('offline-confirm-modal').style.display = 'none';
}

function hideOfflineConfirmOutside(e) {
  if (e.target === document.getElementById('offline-confirm-modal')) hideOfflineConfirm();
}

function confirmOpenControl() {
  hideOfflineConfirm();
  _doOpenControl();
}

async function openControl() {
  let status = _status;
  try {
    const r = await fetch('/api/status');
    status = await r.json();
    _status = status;
    updateStatus();
  } catch (_) {}

  const offline = getOfflineOnboardSystems(status);
  if (offline.length > 0) {
    showOfflineConfirm(offline);
    return;
  }
  _doOpenControl();
}

async function _doOpenControl() {
  await saveConfig();
  activateGamepad();
  document.getElementById('launch').classList.remove('active');
  document.getElementById('control').classList.add('active');
  bindCameraResize();
  setCameraView(_cameraView);
  startHUDLoop();
  _updateGamepadPill();
  loadArmPresets();
  startMissionPoll();
  requestAnimationFrame(() => {
    requestAnimationFrame(() => {
      startAllCameras();
      resizeHUDs();
    });
  });
}

function showLaunch() {
  stopAllCameras();
  stopMissionPoll();
  document.getElementById('control').classList.remove('active');
  document.getElementById('launch').classList.add('active');
  unbindCameraResize();
}

function resizeHUDs() {
  const c = document.getElementById('hud1');
  if (!c) return;
  c.width  = c.parentElement.clientWidth;
  c.height = c.parentElement.clientHeight;
  drawHUD('hud1', _tel);
}

let _hudLoop = null;
function startHUDLoop() {
  if (_hudLoop) return;
  _hudLoop = setInterval(() => {
    drawHUD('hud1', _tel);
  }, 100);
}

// ─────────────────────────────────────────────────────────────
// KEYBINDS MODAL
// ─────────────────────────────────────────────────────────────
function showKeybinds() {
  document.getElementById('keybinds-modal').style.display = 'flex';
}
function hideKeybinds() {
  document.getElementById('keybinds-modal').style.display = 'none';
}
function hideKeybindsOutside(e) {
  if (e.target === document.getElementById('keybinds-modal')) hideKeybinds();
}

// ─────────────────────────────────────────────────────────────
// LOGS
// ─────────────────────────────────────────────────────────────
let _logOpen       = false;
let _logRefreshTimer = null;

// Map JS log name → API endpoint name for onboard (Pi-side) logs
const _onboardLogNames = {
  onboard_stab: 'stab', onboard_arm: 'arm', onboard_cam: 'cam',
  colmap: 'colmap', crabs: 'crabs',
};

function toggleLog() {
  _logOpen = !_logOpen;
  document.getElementById('log-drawer').classList.toggle('open', _logOpen);
  if (_logOpen) {
    refreshLogView();
    _startLogAutoRefresh();
  } else {
    _stopLogAutoRefresh();
  }
}

function switchLog(name) {
  _currentLog = name;
  const tabMap = {
    arm: 'lt-arm', onboard_stab: 'lt-stab', onboard_arm: 'lt-arm2',
    onboard_cam: 'lt-cam', colmap: 'lt-colmap', crabs: 'lt-crabs',
  };
  Object.entries(tabMap).forEach(([n, id]) => {
    const el = document.getElementById(id);
    if (el) el.classList.toggle('active', n === name);
  });
  refreshLogView();
}

function refreshLogView() {
  const apiName = _onboardLogNames[_currentLog];
  if (apiName) {
    // Onboard (Pi-side) logs — fetch from server which SSHes to Pi
    fetch(`/api/onboard_log/${apiName}`)
      .then(r => r.json())
      .then(d => {
        if (d.lines) {
          _logs[_currentLog] = d.lines;
          _renderLogContent(d.lines);
        }
      })
      .catch(() => {
        _renderLogContent(['(Could not fetch log — SSH not connected?)']);
      });
  } else {
    // Local logs (arm_sender) — already in memory from socket events
    _renderLogContent(_logs[_currentLog] || []);
  }
}

function _renderLogContent(lines) {
  const content = document.getElementById('log-content');
  if (!content) return;
  content.innerHTML = lines.map(l =>
    `<div class="log-line">${escapeHtml(l)}</div>`
  ).join('');
  content.scrollTop = content.scrollHeight;
}

function _startLogAutoRefresh() {
  _stopLogAutoRefresh();
  // Refresh onboard logs every 3 s while the drawer is open
  _logRefreshTimer = setInterval(() => {
    if (_logOpen && _onboardLogNames[_currentLog]) refreshLogView();
  }, 3000);
}

function _stopLogAutoRefresh() {
  if (_logRefreshTimer) { clearInterval(_logRefreshTimer); _logRefreshTimer = null; }
}

function appendLogLine(line) {
  if (!_logOpen) return;
  const content = document.getElementById('log-content');
  const div = document.createElement('div');
  div.className   = 'log-line';
  div.textContent = line;
  content.appendChild(div);
  if (content.children.length > 300) content.removeChild(content.firstChild);
  content.scrollTop = content.scrollHeight;
}

function escapeHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// ─────────────────────────────────────────────────────────────
// TOASTS
// ─────────────────────────────────────────────────────────────
function toast(msg, type='') {
  const container = document.getElementById('toast-container');
  const el = document.createElement('div');
  el.className   = 'toast ' + type;
  el.textContent = msg;
  container.appendChild(el);
  setTimeout(() => {
    el.style.opacity    = '0';
    el.style.transition = 'opacity .3s';
    setTimeout(() => el.remove(), 350);
  }, 3500);
}

// ─────────────────────────────────────────────────────────────
// INIT
// ─────────────────────────────────────────────────────────────
window.addEventListener('load', async () => {
  await loadConfig();
  buildArmPwmGrid();
  loadArmPresets();

  // Auto-detect Windows serial port default
  if (navigator.platform.includes('Win') || navigator.userAgent.includes('Windows')) {
    const sp = document.getElementById('cfg-serial_port');
    if (sp && sp.value.startsWith('/dev/')) sp.value = 'COM3';
  }

  // Start gamepad control loop
  requestAnimationFrame(_gamepadControlLoop);
  _updateGamepadPill();

  // Poll status every 2s as WebSocket fallback
  setInterval(() => socketEmit('request_status'), 2000);
  // HTTP fallback when Socket.IO is down
  setInterval(async () => {
    if (socket && socket.connected) return;
    try {
      const r = await fetch('/api/status');
      const d = await r.json();
      _status = d;
      updateStatus();
      if (d.telemetry) { _tel = d.telemetry; updateTelemetry(); }
      if (d.onboard_progress) {
        for (const entry of d.onboard_progress) handleOnboardProgress(entry);
      }
    } catch (_) {}
  }, 2000);
});
