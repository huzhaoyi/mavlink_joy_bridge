const IO_COUNT = 24;

const state = {
  ai: new Array(IO_COUNT).fill(0),
  di: new Array(IO_COUNT).fill(0),
  debounceTimer: null,
};

function setStatus(text, ok) {
  const el = document.getElementById('status');
  el.textContent = text;
  el.className = 'status-text' + (ok === true ? ' ok' : ok === false ? ' err' : '');
  document.getElementById('link_dot').classList.toggle('live', ok === true);
}

async function postIo(extra) {
  const body = {
    udp_target_host: document.getElementById('udp_host').value,
    udp_target_port: parseInt(document.getElementById('udp_port').value, 10),
    send_rate_hz: parseFloat(document.getElementById('send_rate').value),
    auto_send: document.getElementById('auto_send').checked,
    ai: state.ai,
    di: state.di,
    ...extra,
  };
  const res = await fetch('/api/io', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  if (!res.ok) {
    throw new Error('HTTP ' + res.status);
  }
  return res.json();
}

function schedulePost() {
  if (state.debounceTimer) {
    clearTimeout(state.debounceTimer);
  }
  state.debounceTimer = setTimeout(async () => {
    try {
      await postIo({});
      setStatus('已同步', true);
    } catch (err) {
      setStatus('失败: ' + err.message, false);
    }
  }, 50);
}

function buildAiGrid() {
  const grid = document.getElementById('ai_grid');
  for (let i = 0; i < IO_COUNT; i++) {
    const cell = document.createElement('div');
    cell.className = 'io-cell';
    cell.innerHTML =
      '<label>AI' + i +
      ' <span class="val" id="ai_val_' + i + '">0.00</span></label>' +
      '<input type="range" id="ai_' + i + '" min="-1" max="1" step="0.01" value="0">';
    grid.appendChild(cell);
    const slider = cell.querySelector('input');
    slider.addEventListener('input', () => {
      const v = parseFloat(slider.value);
      state.ai[i] = v;
      document.getElementById('ai_val_' + i).textContent = v.toFixed(2);
      schedulePost();
    });
  }
}

function buildDiGrid() {
  const grid = document.getElementById('di_grid');
  for (let i = 0; i < IO_COUNT; i++) {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'di-btn';
    btn.id = 'di_' + i;
    btn.textContent = 'DI' + i;
    btn.addEventListener('click', () => {
      state.di[i] = state.di[i] ? 0 : 1;
      btn.classList.toggle('on', state.di[i] === 1);
      schedulePost();
    });
    grid.appendChild(btn);
  }
}

function syncUiFromState() {
  for (let i = 0; i < IO_COUNT; i++) {
    const slider = document.getElementById('ai_' + i);
    if (slider) {
      slider.value = state.ai[i];
      document.getElementById('ai_val_' + i).textContent = state.ai[i].toFixed(2);
    }
    const btn = document.getElementById('di_' + i);
    if (btn) {
      btn.classList.toggle('on', state.di[i] === 1);
    }
  }
}

async function loadConfig() {
  try {
    const res = await fetch('/api/config');
    if (!res.ok) {
      return;
    }
    const cfg = await res.json();
    document.getElementById('udp_host').value = cfg.udp_target_host || '127.0.0.1';
    document.getElementById('udp_port').value = cfg.udp_target_port || 14550;
    document.getElementById('send_rate').value = cfg.send_rate_hz || 50;
    document.getElementById('auto_send').checked = cfg.auto_send !== false;
    if (cfg.ai && cfg.ai.length) {
      for (let i = 0; i < IO_COUNT; i++) {
        state.ai[i] = cfg.ai[i] || 0;
      }
    }
    if (cfg.di && cfg.di.length) {
      for (let i = 0; i < IO_COUNT; i++) {
        state.di[i] = cfg.di[i] ? 1 : 0;
      }
    }
    syncUiFromState();
    setStatus('已加载', true);
  } catch (_) {
    /* ignore */
  }
}

document.getElementById('btn_apply').addEventListener('click', async () => {
  try {
    await postIo({});
    setStatus('链路已应用', true);
  } catch (err) {
    setStatus('应用失败', false);
  }
});

document.getElementById('btn_zero').addEventListener('click', async () => {
  state.ai.fill(0);
  state.di.fill(0);
  syncUiFromState();
  try {
    await postIo({});
    setStatus('已清零', true);
  } catch (err) {
    setStatus('清零失败', false);
  }
});

document.getElementById('auto_send').addEventListener('change', schedulePost);

buildAiGrid();
buildDiGrid();
loadConfig();
