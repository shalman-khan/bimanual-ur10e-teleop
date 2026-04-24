'use strict';

// ── Constants ─────────────────────────────────────────────────────────────────
const API  = '';
const WS   = `ws://${location.host}/ws`;

const JOINT_LABELS = [
  'Shoulder Pan', 'Shoulder Lift', 'Elbow',
  'Wrist 1', 'Wrist 2', 'Wrist 3'
];

// ── App State ─────────────────────────────────────────────────────────────────
const state = {
  systemState:      'idle',
  mode:             'teleop',
  robotLeftConn:    false,
  robotRightConn:   false,
  gelloLeftConn:    false,
  gelloRightConn:   false,
  ros2Available:    false,
  leftJoints:       Array(6).fill(0),
  leftGripper:      0,
  rightJoints:      Array(6).fill(0),
  rightGripper:     0,
  motionPlanStatus: null,
  settings:         null,
};

// ── WebSocket ─────────────────────────────────────────────────────────────────
let ws = null;

function connectWS() {
  ws = new WebSocket(WS);

  ws.onopen = () => {
    setEl('ws-status', '\u25cf WS connected', 'connected');
  };

  ws.onmessage = ({ data }) => {
    try {
      const msg = JSON.parse(data);
      if (msg.type === 'state_update') handleStateUpdate(msg);
    } catch (e) {}
  };

  ws.onclose = () => {
    setEl('ws-status', '\u25cf WS disconnected', 'disconnected');
    setTimeout(connectWS, 3000);
  };

  ws.onerror = () => ws.close();
}

// ── State Update Handler ──────────────────────────────────────────────────────
function handleStateUpdate(msg) {
  state.systemState    = msg.system_state      || state.systemState;
  state.robotLeftConn  = msg.robot_left_connected  ?? state.robotLeftConn;
  state.robotRightConn = msg.robot_right_connected ?? state.robotRightConn;
  state.gelloLeftConn  = msg.gello_left_connected  ?? state.gelloLeftConn;
  state.gelloRightConn = msg.gello_right_connected ?? state.gelloRightConn;
  state.leftJoints     = msg.left_joints   || state.leftJoints;
  state.leftGripper    = msg.left_gripper  ?? state.leftGripper;
  state.rightJoints    = msg.right_joints  || state.rightJoints;
  state.rightGripper   = msg.right_gripper ?? state.rightGripper;
  state.motionPlanStatus = msg.motion_plan_status ?? state.motionPlanStatus;
  renderUI();
}

// ── Render ────────────────────────────────────────────────────────────────────
function renderUI() {
  const s           = state.systemState;
  const isIdle      = s === 'idle';
  const inTeleop    = s.startsWith('teleop');
  const inMotion    = s.startsWith('motion_plan');
  const connected   = !isIdle;
  const anyRobot    = state.robotLeftConn || state.robotRightConn;
  const anyGello    = state.gelloLeftConn || state.gelloRightConn;
  const isActive    = s === 'teleop_active';
  const executing   = s === 'motion_plan_executing';

  // State badge
  const badge = el('state-badge');
  badge.textContent = s.replace(/_/g, ' ').toUpperCase();
  badge.className   = `badge-${s}`;

  // Header connection dots
  dot('dot-robot-left',  state.robotLeftConn);
  dot('dot-robot-right', state.robotRightConn);
  dot('dot-gello-left',  state.gelloLeftConn);
  dot('dot-gello-right', state.gelloRightConn);

  // ROS2 dot
  const ros2dot = el('dot-ros2');
  if (ros2dot) ros2dot.className = `conn-dot ${state.ros2Available ? 'ok' : 'err'}`;

  // ── Connection card buttons ───────────────────────────────────────────────
  const btnConnRobots = el('btn-connect-robots');
  const btnConnGello  = el('btn-connect-gello');
  const btnDisconn    = el('btn-disconnect');

  if (btnConnRobots) {
    btnConnRobots.textContent = anyRobot ? '\u2714 Robots Connected' : '\u26a1 Connect Robots';
    btnConnRobots.className   = anyRobot ? 'btn-success' : 'btn-primary';
    btnConnRobots.disabled    = executing;
  }
  if (btnConnGello) {
    btnConnGello.textContent = anyGello ? '\u2714 GELLO Connected' : '\uD83C\uDFAE Connect GELLO';
    btnConnGello.className   = anyGello ? 'btn-success btn-sm' : 'btn-secondary btn-sm';
    btnConnGello.disabled    = executing;
  }
  if (btnDisconn) {
    btnDisconn.style.display = connected ? '' : 'none';
  }

  // Hint text
  const hint = el('conn-hint');
  if (hint) {
    if (!anyRobot)         hint.textContent = 'Connect robots first, then GELLO devices.';
    else if (!anyGello)    hint.textContent = 'Robots connected. Connect GELLO to enable teleop.';
    else if (!isActive)    hint.textContent = 'All connected. Click ACTIVE below to start teleoperation.';
    else                   hint.textContent = 'Teleoperation active \u2014 GELLO is driving the arms.';
    hint.style.color = isActive ? 'var(--green)' : 'var(--text-dim)';
  }

  // ── Teleop control buttons ────────────────────────────────────────────────
  const btnPassive = el('btn-passive');
  const btnActive  = el('btn-active');
  const teleopHint = el('teleop-hint');

  if (btnPassive) {
    btnPassive.disabled  = !connected || executing;
    btnPassive.classList.toggle('active', !isActive && connected);
  }
  if (btnActive) {
    btnActive.disabled   = !anyRobot || !anyGello || executing;
    btnActive.classList.toggle('active', isActive);
  }
  if (teleopHint) {
    if (!anyRobot) {
      teleopHint.innerHTML = 'Connect robots first.';
    } else if (!anyGello) {
      teleopHint.innerHTML = 'Connect GELLO to enable <strong style="color:var(--green)">ACTIVE</strong> mode.';
    } else if (!isActive) {
      teleopHint.innerHTML = 'Ready \u2014 click <strong style="color:var(--green)">ACTIVE</strong> to start GELLO teleoperation.';
    } else {
      teleopHint.innerHTML = '<span style="color:var(--green)">\u25cf ACTIVE</span> \u2014 GELLO is driving the arms in real time.';
    }
  }

  // ── Mode tabs ─────────────────────────────────────────────────────────────
  el('tab-teleop').classList.toggle('active',   state.mode === 'teleop');
  el('tab-motion').classList.toggle('active',   state.mode === 'motion_plan');
  el('tab-gripper').classList.toggle('active',  state.mode === 'gripper');

  // ── Panel visibility ──────────────────────────────────────────────────────
  el('motion-plan-panel').style.display = state.mode === 'motion_plan' ? 'block' : 'none';
  el('gripper-panel').style.display     = state.mode === 'gripper'     ? 'block' : 'none';

  // ── Gripper panel ─────────────────────────────────────────────────────────
  const canGripper = anyRobot && !isIdle && !executing;
  const leftClosed  = (state.leftGripper  || 0) > 0.5;
  const rightClosed = (state.rightGripper || 0) > 0.5;

  const bglOpen  = el('btn-gripper-left-open');
  const bglClose = el('btn-gripper-left-close');
  const bgrOpen  = el('btn-gripper-right-open');
  const bgrClose = el('btn-gripper-right-close');

  if (bglOpen)  { bglOpen.disabled  = !canGripper; bglOpen.classList.toggle('active',  !leftClosed); }
  if (bglClose) { bglClose.disabled = !canGripper; bglClose.classList.toggle('active', leftClosed); }
  if (bgrOpen)  { bgrOpen.disabled  = !canGripper; bgrOpen.classList.toggle('active',  !rightClosed); }
  if (bgrClose) { bgrClose.disabled = !canGripper; bgrClose.classList.toggle('active', rightClosed); }

  // Gripper progress bars in gripper panel
  const lPct = ((state.leftGripper  || 0) * 100).toFixed(0);
  const rPct = ((state.rightGripper || 0) * 100).toFixed(0);
  const gbl = el('gripper-bar-left');   if (gbl) gbl.style.width = lPct + '%';
  const gbr = el('gripper-bar-right');  if (gbr) gbr.style.width = rPct + '%';
  const gpl = el('gripper-pct-left');   if (gpl) gpl.textContent = lPct + '%';
  const gpr = el('gripper-pct-right');  if (gpr) gpr.textContent = rPct + '%';

  // Execute button
  const canPlan = (s === 'motion_plan_idle' || s === 'teleop_passive') && connected;
  const btnExec = el('btn-execute-move');
  if (btnExec) {
    btnExec.disabled    = !canPlan;
    btnExec.textContent = executing ? '\u23f3 Executing\u2026' : '\u25b6 Execute Move';
  }

  // Motion status
  const msEl = el('motion-status');
  if (msEl && state.motionPlanStatus) {
    msEl.textContent = `Status: ${state.motionPlanStatus}`;
    msEl.style.color = state.motionPlanStatus === 'completed' ? 'var(--green)'
                     : state.motionPlanStatus === 'failed'    ? 'var(--red)'
                     : 'var(--amber)';
  }

  // ROS2 status
  const ros2El = el('ros2-node-status');
  if (ros2El) {
    ros2El.textContent = state.ros2Available ? '\u2713 Active' : '\u2717 Not available';
    ros2El.style.color = state.ros2Available ? 'var(--green)' : 'var(--red)';
  }

  // Settings IP labels
  if (state.settings) {
    el('left-ip-label').textContent  = state.settings.robot_left_ip  || '';
    el('right-ip-label').textContent = state.settings.robot_right_ip || '';
  }

  // Joint tables
  renderJointTable('left-joints-body',  state.leftJoints,  state.leftGripper);
  renderJointTable('right-joints-body', state.rightJoints, state.rightGripper);
}

function renderJointTable(tbodyId, joints, gripper) {
  const tbody = el(tbodyId);
  if (!tbody) return;
  let html = '';
  for (let i = 0; i < 6; i++) {
    const rad = joints[i] || 0;
    const deg = (rad * 180 / Math.PI).toFixed(1);
    html += `<tr>
      <td class="joint-label">J${i+1} \u2014 ${JOINT_LABELS[i].split(' ').pop()}</td>
      <td class="joint-rad">${rad.toFixed(4)}</td>
      <td class="joint-deg">${deg}\u00b0</td>
    </tr>`;
  }
  const gripPct = ((gripper || 0) * 100).toFixed(0);
  html += `<tr class="gripper-row">
    <td class="joint-label">Gripper</td>
    <td class="joint-rad">${(gripper||0).toFixed(3)}</td>
    <td class="joint-deg">${gripPct}%</td>
  </tr>`;
  html += `<tr><td colspan="3">
    <div class="gripper-bar"><div class="gripper-fill" style="width:${gripPct}%"></div></div>
  </td></tr>`;
  tbody.innerHTML = html;
}

// ── Target Joint Inputs ───────────────────────────────────────────────────────
function buildTargetInputs() {
  ['left', 'right'].forEach(side => {
    const container = el(`${side}-target-inputs`);
    if (!container) return;
    let html = '';
    for (let i = 0; i < 6; i++) {
      html += `<div class="joint-input-row">
        <label>J${i+1} ${JOINT_LABELS[i].split(' ').pop()}</label>
        <input type="number" id="tgt-${side}-j${i}" value="0.0000" step="0.01" />
        <span class="unit">rad</span>
      </div>`;
    }
    html += `<div class="joint-input-row">
      <label>Gripper</label>
      <input type="number" id="tgt-${side}-gripper" value="0.0" min="0" max="1" step="0.05" />
      <span class="unit">0\u20131</span>
    </div>`;
    container.innerHTML = html;
  });
}

function getTargetJoints(side) {
  const joints = [];
  for (let i = 0; i < 6; i++) {
    joints.push(parseFloat(el(`tgt-${side}-j${i}`)?.value) || 0);
  }
  const gripper = parseFloat(el(`tgt-${side}-gripper`)?.value) || 0;
  return { joints, gripper };
}

// ── App Actions ───────────────────────────────────────────────────────────────
const app = {

  async connectRobots() {
    statusMsg('Connecting to robots\u2026');
    el('btn-connect-robots').disabled = true;
    const res = await post('/api/connect/robots');
    el('btn-connect-robots').disabled = false;
    if (res.result) {
      const l = res.result.left  || 'unknown';
      const r = res.result.right || 'unknown';
      state.robotLeftConn  = l === 'connected';
      state.robotRightConn = r === 'connected';
      if (res.state) state.systemState = res.state;
      const ok = !l.includes('error') && !r.includes('error');
      toast(`Robots: L=${l} | R=${r}`, ok ? 'success' : 'error');
    }
    await app.fetchSettings();
    renderUI();
  },

  async connectGello() {
    statusMsg('Connecting to GELLO devices\u2026');
    el('btn-connect-gello').disabled = true;
    const res = await post('/api/connect/gello');
    el('btn-connect-gello').disabled = false;
    if (res.result) {
      const l = res.result.left  || 'unknown';
      const r = res.result.right || 'unknown';
      state.gelloLeftConn  = l === 'connected' || l.includes('mock');
      state.gelloRightConn = r === 'connected' || r.includes('mock');
      const ok = !l.includes('error') && !r.includes('error');
      toast(`GELLO: L=${l} | R=${r}`, ok ? 'success' : 'error');
    }
    renderUI();
  },

  async disconnect() {
    if (!confirm('Disconnect from robots and GELLO?')) return;
    await post('/api/disconnect');
    state.robotLeftConn  = false;
    state.robotRightConn = false;
    state.gelloLeftConn  = false;
    state.gelloRightConn = false;
    state.systemState    = 'idle';
    state.mode           = 'teleop';
    toast('Disconnected', 'info');
    renderUI();
  },

  async switchMode(mode) {
    if (state.systemState === 'idle') {
      toast('Connect to robots first', 'error');
      return;
    }
    state.mode = mode;

    if (mode === 'gripper') {
      // Arms must be passive so they hold position while gripper is controlled.
      // Transition TELEOP_ACTIVE → TELEOP_PASSIVE silently.
      if (state.systemState === 'teleop_active') {
        const res = await post('/api/teleop/state', { state: 'passive' });
        if (res.state) state.systemState = res.state;
      }
    } else {
      const res = await post('/api/mode', { mode });
      if (res.error) toast(res.error, 'error');
      if (res.state) state.systemState = res.state;
    }
    renderUI();
  },

  async setTeleopState(s) {
    const res = await post('/api/teleop/state', { state: s });
    if (res.error) {
      toast(res.error, 'error');
    } else if (res.state) {
      state.systemState = res.state;
      renderUI();
    }
  },

  async executeMoveToTarget() {
    const left  = getTargetJoints('left');
    const right = getTargetJoints('right');
    const speed = parseFloat(el('movej-speed')?.value) || 0.5;
    const accel = parseFloat(el('movej-accel')?.value) || 0.3;
    const res = await post('/api/motion_plan/move_to', {
      left_joints:  left.joints,  left_gripper:  left.gripper,
      right_joints: right.joints, right_gripper: right.gripper,
      speed, acceleration: accel,
    });
    if (res.error) toast(res.error, 'error');
    else toast('Move executing\u2026', 'info');
  },

  fillCurrentJoints() {
    for (let i = 0; i < 6; i++) {
      const lEl = el(`tgt-left-j${i}`);
      const rEl = el(`tgt-right-j${i}`);
      if (lEl) lEl.value = (state.leftJoints[i]  || 0).toFixed(4);
      if (rEl) rEl.value = (state.rightJoints[i] || 0).toFixed(4);
    }
    const lgEl = el('tgt-left-gripper');
    const rgEl = el('tgt-right-gripper');
    if (lgEl) lgEl.value = (state.leftGripper  || 0).toFixed(3);
    if (rgEl) rgEl.value = (state.rightGripper || 0).toFixed(3);
    toast('Filled from current joint state', 'info');
  },

  async executeTrajectory() {
    const raw = el('traj-textarea')?.value?.trim();
    if (!raw) { toast('Enter trajectory JSON first', 'error'); return; }
    let points;
    try {
      points = JSON.parse(raw);
      if (!Array.isArray(points) || !points.length) throw new Error('Must be a non-empty array');
    } catch (e) {
      toast(`JSON parse error: ${e.message}`, 'error');
      return;
    }
    const res = await post('/api/motion_plan/execute_trajectory', { points });
    if (res.error) toast(res.error, 'error');
    else toast(`Trajectory executing (${res.num_points} points)\u2026`, 'info');
  },

  loadTrajExample() {
    el('traj-textarea').value = JSON.stringify([
      { left_joints:[0,-1.571,-1.571,-1.571,1.571,0],  left_gripper:0.0,
        right_joints:[0,-1.571,1.571,-1.571,-1.571,0], right_gripper:0.0, time_from_start:0.0 },
      { left_joints:[0.1,-1.471,-1.571,-1.571,1.571,0], left_gripper:0.5,
        right_joints:[-0.1,-1.471,1.571,-1.571,-1.571,0], right_gripper:0.5, time_from_start:2.0 },
    ], null, 2);
  },

  async _doSetGripper(side, position) {
    const res = await post('/api/gripper', { side, position });
    if (res.error) {
      toast(res.error, 'error');
    } else {
      if (side === 'left'  || side === 'both') state.leftGripper  = position;
      if (side === 'right' || side === 'both') state.rightGripper = position;
      renderUI();
    }
  },

  setGripper(side, position) {
    const delaySec = Math.max(0, parseFloat(el('gripper-delay')?.value) || 0);
    const label    = side.charAt(0).toUpperCase() + side.slice(1);
    const action   = position <= 0 ? 'OPEN' : 'CLOSE';

    if (delaySec <= 0) {
      app._doSetGripper(side, position);
      return;
    }

    toast(`${label} gripper ${action} in ${delaySec}s\u2026`, 'info', delaySec * 1000 + 500);

    setTimeout(() => {
      toast(`${label} gripper ${action} — executing`, 'success');
      app._doSetGripper(side, position);
    }, delaySec * 1000);
  },

  async fetchSettings() {
    const s = await get('/api/settings');
    state.settings = s;
    return s;
  },

  async openSettings() {
    const s = await app.fetchSettings();
    setInput('s-robot-left-ip',    s.robot_left_ip);
    setInput('s-robot-right-ip',   s.robot_right_ip);
    setInput('s-gello-left-port',  s.gello_left_port);
    setInput('s-gello-right-port', s.gello_right_port);
    setInput('s-ctrl-hz',          s.control_rate_hz);
    setInput('s-sv-vel',           s.servoj_velocity);
    setInput('s-sv-acc',           s.servoj_acceleration);
    setInput('s-sv-look',          s.servoj_lookahead_time);
    setInput('s-sv-gain',          s.servoj_gain);
    setInput('s-mj-speed',         s.movej_speed);
    setInput('s-mj-accel',         s.movej_acceleration);
    const lcfg = el('s-gello-left-cfg');
    const rcfg = el('s-gello-right-cfg');
    if (lcfg) lcfg.value = JSON.stringify(s.gello_left_config,  null, 2);
    if (rcfg) rcfg.value = JSON.stringify(s.gello_right_config, null, 2);
    el('settings-modal').classList.add('open');
  },

  closeSettings() {
    el('settings-modal').classList.remove('open');
  },

  async saveSettings() {
    const updates = {
      robot_left_ip:         getInput('s-robot-left-ip'),
      robot_right_ip:        getInput('s-robot-right-ip'),
      gello_left_port:       getInput('s-gello-left-port'),
      gello_right_port:      getInput('s-gello-right-port'),
      control_rate_hz:       parseFloat(getInput('s-ctrl-hz')),
      servoj_velocity:       parseFloat(getInput('s-sv-vel')),
      servoj_acceleration:   parseFloat(getInput('s-sv-acc')),
      servoj_lookahead_time: parseFloat(getInput('s-sv-look')),
      servoj_gain:           parseFloat(getInput('s-sv-gain')),
      movej_speed:           parseFloat(getInput('s-mj-speed')),
      movej_acceleration:    parseFloat(getInput('s-mj-accel')),
    };
    try {
      const lcfg = el('s-gello-left-cfg')?.value;
      const rcfg = el('s-gello-right-cfg')?.value;
      if (lcfg) updates.gello_left_config  = JSON.parse(lcfg);
      if (rcfg) updates.gello_right_config = JSON.parse(rcfg);
    } catch (e) {
      toast(`GELLO config JSON error: ${e.message}`, 'error');
      return;
    }
    const res = await post('/api/settings', updates);
    if (res.status === 'saved') {
      state.settings = res.settings;
      toast('Settings saved', 'success');
      app.closeSettings();
    } else {
      toast('Save failed', 'error');
    }
  },
};

// ── HTTP helpers ──────────────────────────────────────────────────────────────
async function post(path, body = {}) {
  try {
    const r = await fetch(API + path, {
      method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(body),
    });
    return r.json();
  } catch (e) {
    toast(`Request failed: ${e.message}`, 'error');
    return { error: e.message };
  }
}

async function get(path) {
  try { return (await fetch(API + path)).json(); }
  catch (e) { return {}; }
}

// ── DOM helpers ───────────────────────────────────────────────────────────────
function el(id)              { return document.getElementById(id); }
function setEl(id, txt, cls) { const e = el(id); if (!e) return; if (txt !== undefined) e.textContent = txt; if (cls !== undefined) e.className = cls; }
function getInput(id)        { return el(id)?.value ?? ''; }
function setInput(id, v)     { const e = el(id); if (e) e.value = v ?? ''; }

function dot(id, connected) {
  const e = el(id);
  if (!e) return;
  e.className = `conn-dot ${connected ? 'ok' : 'err'}`;
}

function statusMsg(msg) { setEl('status-msg', msg); }

function toast(msg, type = 'info', durationMs = 4000) {
  const c = el('toast-container');
  const d = document.createElement('div');
  d.className   = `toast ${type}`;
  d.textContent = msg;
  c.appendChild(d);
  setTimeout(() => d.remove(), durationMs);
}

// ── Init ─────────────────────────────────────────────────────────────────────
(async function init() {
  buildTargetInputs();
  connectWS();

  await app.fetchSettings();

  const status = await get('/api/status');
  if (status.ros2_available        !== undefined) state.ros2Available  = status.ros2_available;
  if (status.robot_left_connected  !== undefined) state.robotLeftConn  = status.robot_left_connected;
  if (status.robot_right_connected !== undefined) state.robotRightConn = status.robot_right_connected;
  if (status.gello_left_connected  !== undefined) state.gelloLeftConn  = status.gello_left_connected;
  if (status.gello_right_connected !== undefined) state.gelloRightConn = status.gello_right_connected;
  if (status.system_state          !== undefined) state.systemState    = status.system_state;
  if (status.left_joints)  state.leftJoints  = status.left_joints;
  if (status.right_joints) state.rightJoints = status.right_joints;

  renderUI();

  // Polling fallback: keeps the display fresh even when WS hiccups.
  // The WS is the primary real-time path; this fills any gaps.
  setInterval(async () => {
    const status = await get('/api/status');
    if (status.system_state !== undefined) {
      handleStateUpdate({ type: 'state_update', ...status });
    }
  }, 1000);
})();
