const IO_COUNT = 24;
const ADC_MIN_V = 0.0;
const ADC_MAX_V = 5.0;
const ADC_CENTER_V = 2.5;

const state = {
  adc: new Array(IO_COUNT).fill(ADC_CENTER_V),
  gpio: new Array(IO_COUNT).fill(0),
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
    adc: state.adc,
    gpio: state.gpio,
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

function buildAdcGrid() {
  const grid = document.getElementById('adc_grid');
  for (let i = 0; i < IO_COUNT; i++) {
    const cell = document.createElement('div');
    cell.className = 'io-cell';
    cell.innerHTML =
      '<label>ADC' + i +
      ' <span class="val" id="adc_val_' + i + '">2.50</span> V</label>' +
      '<input type="range" id="adc_' + i + '" min="0" max="5" step="0.01" value="2.5">';
    grid.appendChild(cell);
    const slider = cell.querySelector('input');
    slider.addEventListener('input', () => {
      const v = parseFloat(slider.value);
      state.adc[i] = v;
      document.getElementById('adc_val_' + i).textContent = v.toFixed(2);
      schedulePost();
    });
  }
}

function buildGpioGrid() {
  const grid = document.getElementById('gpio_grid');
  for (let i = 0; i < IO_COUNT; i++) {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'di-btn';
    btn.id = 'gpio_' + i;
    btn.textContent = 'GPIO' + i;
    btn.addEventListener('click', () => {
      state.gpio[i] = state.gpio[i] ? 0 : 1;
      btn.classList.toggle('on', state.gpio[i] === 1);
      schedulePost();
    });
    grid.appendChild(btn);
  }
}

function syncUiFromState() {
  for (let i = 0; i < IO_COUNT; i++) {
    const slider = document.getElementById('adc_' + i);
    if (slider) {
      slider.value = state.adc[i];
      document.getElementById('adc_val_' + i).textContent = state.adc[i].toFixed(2);
    }
    const btn = document.getElementById('gpio_' + i);
    if (btn) {
      btn.classList.toggle('on', state.gpio[i] === 1);
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
    if (cfg.adc && cfg.adc.length) {
      for (let i = 0; i < IO_COUNT; i++) {
        state.adc[i] = cfg.adc[i] != null ? cfg.adc[i] : ADC_CENTER_V;
      }
    }
    if (cfg.gpio && cfg.gpio.length) {
      for (let i = 0; i < IO_COUNT; i++) {
        state.gpio[i] = cfg.gpio[i] ? 1 : 0;
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

document.getElementById('btn_center_adc').addEventListener('click', async () => {
  state.adc.fill(ADC_CENTER_V);
  syncUiFromState();
  try {
    await postIo({});
    setStatus('ADC 已居中', true);
  } catch (err) {
    setStatus('操作失败', false);
  }
});

document.getElementById('btn_zero').addEventListener('click', async () => {
  state.adc.fill(ADC_MIN_V);
  state.gpio.fill(0);
  syncUiFromState();
  try {
    await postIo({});
    setStatus('已清零', true);
  } catch (err) {
    setStatus('清零失败', false);
  }
});

document.getElementById('auto_send').addEventListener('change', schedulePost);

buildAdcGrid();
buildGpioGrid();
loadConfig();
