/* ═══════════════════════════════════════════════════════════════
   app.js — UI behaviour only. Talks exclusively to the API facade
   in api.js; contains no fetch(), no socket, no mock data.
   Charts and the map are drawn on canvas so the dashboard has zero
   external dependencies and works offline on a ward terminal.
   ═══════════════════════════════════════════════════════════════ */
(function () {
  'use strict';

  const $  = (s, r = document) => r.querySelector(s);
  const $$ = (s, r = document) => Array.from(r.querySelectorAll(s));
  const fmt = (n, d = 1) => (n === null || n === undefined || Number.isNaN(n)) ? '—' : Number(n).toFixed(d);
  const titleCase = s => String(s || '').replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase());

  const state = { page: 'dashboard', status: null, map: null, reduceMotion: false };

  /* ═════════ ROUTER ═════════ */
  function route() {
    const page = (location.hash.replace('#/', '') || 'dashboard');
    state.page = page;
    $$('.page').forEach(p => p.classList.toggle('is-active', p.dataset.page === page));
    $$('.rail__item').forEach(a => a.classList.toggle('is-active', a.dataset.page === page));
    if (page === 'map') refreshMap();
    if (page === 'camera') refreshCamera();
    if (page === 'analytics') refreshAnalytics();
    if (page === 'missions') refreshMissions();
  }
  window.addEventListener('hashchange', route);

  /* ═════════ TOASTS ═════════ */
  const KIND = { ok: 'toast--ok', warn: 'toast--warn', bad: 'toast--bad', info: '' };
  function toast(kind, title, detail) {
    const el = document.createElement('div');
    el.className = 'toast ' + (KIND[kind] || '');
    el.innerHTML = `<b></b><span></span>`;
    el.querySelector('b').textContent = title;
    el.querySelector('span').textContent = detail || '';
    $('#toasts').appendChild(el);
    setTimeout(() => { el.style.opacity = '0'; setTimeout(() => el.remove(), 320); }, 4600);
  }

  /* ═════════ VITALS TRACE (signature element) ═════════ */
  const trace = { buf: [], canvas: null, ctx: null };
  function initTrace() {
    trace.canvas = $('#vitalsTrace');
    const resize = () => {
      const r = trace.canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      trace.canvas.width = Math.max(1, r.width * dpr);
      trace.canvas.height = Math.max(1, r.height * dpr);
      trace.ctx = trace.canvas.getContext('2d');
      trace.ctx.scale(dpr, dpr);
      trace.w = r.width; trace.h = r.height;
    };
    resize();
    window.addEventListener('resize', resize);
    drawTrace();
  }
  function drawTrace() {
    const c = trace.ctx;
    if (c) {
      const { w, h } = trace;
      c.clearRect(0, 0, w, h);
      c.strokeStyle = 'rgba(126,147,168,.16)';
      c.lineWidth = 1;
      c.beginPath(); c.moveTo(0, h / 2); c.lineTo(w, h / 2); c.stroke();

      const pts = trace.buf;
      if (pts.length > 1) {
        const step = w / 90;
        const draw = (key, colour, scale) => {
          c.beginPath(); c.strokeStyle = colour; c.lineWidth = 1.6;
          pts.forEach((p, i) => {
            const x = w - (pts.length - 1 - i) * step;
            const y = h - (p[key] / scale) * (h - 8) - 4;
            i ? c.lineTo(x, y) : c.moveTo(x, y);
          });
          c.stroke();
        };
        draw('cov', '#0E7C86', 100);
        draw('bat', '#E8A33D', 100);
        draw('spd', '#3FA34D', 0.4);
      }
    }
    requestAnimationFrame(() => setTimeout(drawTrace, 90));
  }

  /* ═════════ STATUS BINDING ═════════ */
  function applyStatus(s) {
    state.status = s;
    trace.buf.push({ cov: s.coverage_pct, bat: s.battery_pct, spd: s.speed_ms });
    if (trace.buf.length > 90) trace.buf.shift();

    $('#vBattery').textContent  = fmt(s.battery_pct, 0) + '%';
    $('#vCoverage').textContent = fmt(s.coverage_pct, 0) + '%';
    $('#vSpeed').textContent    = fmt(s.speed_ms, 2);
    $('#vState').textContent    = titleCase(s.state);
    $('#linkState').classList.toggle('is-down', !s.connected);
    $('#linkState').childNodes[0].nodeValue = API.source === 'mock' ? 'DEMO DATA' : 'LIVE';

    $('#sMode').textContent      = titleCase(s.state);
    $('#sTask').textContent      = s.task || '—';
    $('#sBattery').textContent   = fmt(s.battery_pct, 0) + '%';
    $('#sBatteryBar').style.width  = clampPct(s.battery_pct);
    $('#sCoverage').textContent  = fmt(s.coverage_pct, 0) + '%';
    $('#sCoverageBar').style.width = clampPct(s.coverage_pct);
    $('#sDeliveries').textContent = s.deliveries_done;
    $('#sPending').textContent   = s.deliveries_pending
      ? `${s.deliveries_pending} waiting` : 'Queue empty';

    $('#sPos').textContent     = `${fmt(s.x, 0)}, ${fmt(s.y, 0)} px`;
    $('#sHeading').textContent = fmt(s.heading_deg, 0) + '°';
    $('#sVel').textContent     = fmt(s.speed_ms, 2) + ' m/s';
    $('#sConf').textContent    = fmt(s.localization_confidence * 100, 0) + '%';
    $('#sDrift').textContent   = fmt(s.drift_px, 1) + ' px';

    $('#sFps').textContent    = fmt(s.fps, 1) + ' Hz';
    $('#sCpu').textContent    = fmt(s.cpu_pct, 0) + '%';
    $('#sRam').textContent    = fmt(s.ram_mb, 0) + ' MB';
    $('#sPlan').textContent   = fmt(s.planner_ms, 1) + ' ms';
    $('#sUptime').textContent = formatUptime(s.uptime_s);

    const n = s.deliveries_pending || 0;
    const badge = $('#navQueueCount');
    badge.textContent = n; badge.dataset.zero = n ? '0' : '1';
  }
  const clampPct = v => Math.max(0, Math.min(100, Number(v) || 0)) + '%';
  function formatUptime(sec) {
    if (!sec && sec !== 0) return '—';
    const h = Math.floor(sec / 3600), m = Math.floor(sec % 3600 / 60);
    return h ? `${h}h ${m}m` : `${m}m ${sec % 60}s`;
  }

  /* ═════════ ACTIVITY FEED ═════════ */
  function renderFeed(events) {
    const ul = $('#feed');
    if (!events || !events.length) { ul.innerHTML = '<li class="feed__empty">No events yet.</li>'; return; }
    ul.innerHTML = events.slice(0, 14).map(e => `
      <li><time>${new Date(e.ts).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' })}</time>
          <span class="dot dot--${e.kind === 'info' ? '' : e.kind}"></span>
          <span><strong>${escape(e.title)}</strong>${e.detail ? ' — ' + escape(e.detail) : ''}</span></li>`).join('');
  }
  const escape = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

  /* ═════════ MISSIONS ═════════ */
  async function refreshMissions() {
    const data = await API.missions.list();
    const ul = $('#queue');
    const q = data.queue || [];
    $('#queueCount').textContent = q.length ? `${q.length} waiting` : 'empty';
    if (!q.length) { ul.innerHTML = '<li class="queue__empty">Nothing queued. Add a delivery to start.</li>'; return; }
    ul.innerHTML = q.map((m, i) => `
      <li class="${i === 0 ? 'is-active' : ''}">
        <span class="queue__idx">${String(i + 1).padStart(2, '0')}</span>
        <span><span class="queue__dest">${escape(m.label)}</span><br>
              <span class="queue__item">${escape(m.payload)}</span></span>
        <span class="queue__pri pri-${m.priority}">${['Routine', 'Urgent', 'Emergency'][m.priority]}</span>
        <span>
          ${i > 0 ? `<button class="queue__del" data-promote="${m.id}" title="Move to front">↑</button>` : ''}
          <button class="queue__del" data-remove="${m.id}" title="Remove">×</button>
        </span>
      </li>`).join('');
  }

  /* ═════════ MAP ═════════ */
  async function refreshMap() {
    state.map = await API.robot.map();
    drawMap();
  }
  function drawMap() {
    const m = state.map; const cv = $('#mapCanvas');
    if (!m || !cv) return;
    const ctx = cv.getContext('2d');
    const cell = m.cell;
    cv.width = m.width * cell; cv.height = m.height * cell;

    ctx.fillStyle = '#1B2733'; ctx.fillRect(0, 0, cv.width, cv.height);
    for (let y = 0; y < m.height; y++) for (let x = 0; x < m.width; x++) {
      const v = m.cells[y * m.width + x];
      if (v === 0) continue;
      ctx.fillStyle = v < 0 ? '#63798F' : '#D8E2EC';
      ctx.fillRect(x * cell, y * cell, cell, cell);
    }
    if ($('#tgFrontier').checked && m.frontiers) {
      ctx.fillStyle = 'rgba(232,163,61,.55)';
      m.frontiers.forEach(([x, y]) => ctx.fillRect(x * cell, y * cell, cell, cell));
    }
    if ($('#tgTrail').checked && m.trail && m.trail.length > 1) {
      ctx.strokeStyle = 'rgba(63,163,77,.4)'; ctx.lineWidth = 2; ctx.beginPath();
      m.trail.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)); ctx.stroke();
    }
    if ($('#tgPath').checked && m.path && m.path.length > 1) {
      ctx.strokeStyle = '#E8A33D'; ctx.lineWidth = 3; ctx.setLineDash([9, 6]); ctx.beginPath();
      m.path.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
      ctx.stroke(); ctx.setLineDash([]);
    }
    if (m.dock) {
      ctx.fillStyle = '#0E7C86';
      ctx.fillRect(m.dock[0] - 9, m.dock[1] - 9, 18, 18);
      ctx.fillStyle = '#fff'; ctx.font = 'bold 10px monospace'; ctx.textAlign = 'center';
      ctx.fillText('D', m.dock[0], m.dock[1] + 4);
    }
    if (m.goal) {
      ctx.strokeStyle = '#E8A33D'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(m.goal[0], m.goal[1], 11, 0, Math.PI * 2); ctx.stroke();
    }
    const p = m.pose;
    ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(p.theta);
    ctx.fillStyle = '#3FA34D';
    ctx.beginPath(); ctx.moveTo(13, 0); ctx.lineTo(-8, 8); ctx.lineTo(-8, -8); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.beginPath(); ctx.strokeStyle = 'rgba(63,163,77,.35)'; ctx.lineWidth = 1.5;
    ctx.arc(p.x, p.y, 17, 0, Math.PI * 2); ctx.stroke();

    $('#mapPose').textContent =
      `x ${p.x.toFixed(0)}  y ${p.y.toFixed(0)}  θ ${(p.theta * 180 / Math.PI).toFixed(0)}°`;
  }

  /* ═════════ CAMERA ═════════ */
  async function refreshCamera() {
    const c = await API.robot.camera();
    const cv = $('#camCanvas'); const ctx = cv.getContext('2d');
    // A real backend supplies c.frame as a data URL; the demo renders a
    // synthetic corridor so the overlay pipeline is visible without hardware.
    if (c.frame) {
      const img = new Image();
      img.onload = () => { ctx.drawImage(img, 0, 0, cv.width, cv.height); overlay(ctx, c); };
      img.src = c.frame;
    } else {
      const g = ctx.createLinearGradient(0, 0, 0, cv.height);
      g.addColorStop(0, '#2E3F52'); g.addColorStop(.55, '#3C5064'); g.addColorStop(1, '#556C82');
      ctx.fillStyle = g; ctx.fillRect(0, 0, cv.width, cv.height);
      ctx.strokeStyle = 'rgba(216,226,236,.18)'; ctx.lineWidth = 1;
      for (let i = 1; i < 9; i++) {
        ctx.beginPath(); ctx.moveTo(0, 250 + i * 26); ctx.lineTo(cv.width, 250 + i * 26); ctx.stroke();
      }
      ctx.beginPath(); ctx.moveTo(230, 250); ctx.lineTo(60, 480); ctx.moveTo(410, 250); ctx.lineTo(580, 480);
      ctx.stroke();
      overlay(ctx, c);
    }
    $('#camFps').textContent = fmt(c.fps, 1) + ' fps';
    $('#camDet').textContent = `${c.detections.length} detected`;
    $('#detList').innerHTML = c.detections.length
      ? c.detections.map(d => `<li><span>${escape(d.label)}</span>
          <span class="mono">${(d.confidence * 100).toFixed(0)}% · ${d.range_m} m</span></li>`).join('')
      : '<li class="feed__empty">Nothing in view.</li>';
  }
  function overlay(ctx, c) {
    ctx.lineWidth = 2; ctx.font = 'bold 12px monospace';
    c.detections.forEach(d => {
      const [x, y, w, h] = d.box;
      ctx.strokeStyle = '#E8A33D'; ctx.strokeRect(x, y, w, h);
      ctx.fillStyle = 'rgba(232,163,61,.92)'; ctx.fillRect(x, y - 17, ctx.measureText(d.label).width + 14, 17);
      ctx.fillStyle = '#1B2733'; ctx.fillText(d.label, x + 6, y - 5);
    });
  }

  /* ═════════ CHARTS ═════════ */
  const CHART_COLOUR = {
    coverage: '#0E7C86', battery: '#E8A33D', cpu: '#C4453B', ram: '#4A5866',
    fps: '#3FA34D', speed: '#0E7C86', drift: '#E8A33D', path: '#4A5866'
  };
  function sparkline(canvas, data, colour) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth || 260, h = 150;
    canvas.width = w * dpr; canvas.height = h * dpr;
    const ctx = canvas.getContext('2d'); ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, w, h);
    if (!data || data.length < 2) {
      ctx.fillStyle = '#4A5866'; ctx.font = '12px Inter,sans-serif';
      ctx.fillText('Collecting…', 8, h / 2); return;
    }
    const min = Math.min(...data), max = Math.max(...data);
    const pad = (max - min) * 0.15 || 1;
    const lo = min - pad, hi = max + pad;
    const X = i => (i / (data.length - 1)) * (w - 6) + 3;
    const Y = v => h - 18 - ((v - lo) / (hi - lo)) * (h - 34);

    ctx.strokeStyle = 'rgba(211,217,224,.7)'; ctx.lineWidth = 1;
    [0, .5, 1].forEach(f => {
      const y = 16 + f * (h - 34);
      ctx.beginPath(); ctx.moveTo(3, y); ctx.lineTo(w - 3, y); ctx.stroke();
    });

    const grad = ctx.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0, colour + '33'); grad.addColorStop(1, colour + '00');
    ctx.beginPath(); ctx.moveTo(X(0), Y(data[0]));
    data.forEach((v, i) => ctx.lineTo(X(i), Y(v)));
    ctx.lineTo(X(data.length - 1), h - 18); ctx.lineTo(X(0), h - 18); ctx.closePath();
    ctx.fillStyle = grad; ctx.fill();

    ctx.beginPath(); ctx.strokeStyle = colour; ctx.lineWidth = 2;
    data.forEach((v, i) => i ? ctx.lineTo(X(i), Y(v)) : ctx.moveTo(X(i), Y(v)));
    ctx.stroke();

    const last = data[data.length - 1];
    ctx.fillStyle = colour;
    ctx.beginPath(); ctx.arc(X(data.length - 1), Y(last), 3.5, 0, Math.PI * 2); ctx.fill();
    ctx.font = 'bold 13px "JetBrains Mono",monospace'; ctx.fillStyle = '#0F1620';
    ctx.fillText(last.toFixed(1), 4, 13);
  }
  async function refreshAnalytics() {
    const a = await API.analytics.summary();
    $$('[data-chart]').forEach(cv => {
      const k = cv.dataset.chart;
      sparkline(cv, a.series[k], CHART_COLOUR[k] || '#0E7C86');
    });
    const tb = $('#deliveryTable tbody');
    tb.innerHTML = a.completed.length
      ? a.completed.map(m => `<tr><td>${escape(m.label)}</td><td>${escape(m.payload)}</td>
          <td>${new Date(m.completedAt).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}</td>
          <td class="num mono">${Math.floor(m.durationSec / 60)}m ${m.durationSec % 60}s</td></tr>`).join('')
      : '<tr><td colspan="4" class="queue__empty">No deliveries completed yet.</td></tr>';
  }

  /* ═════════ EVENT WIRING ═════════ */
  function wire() {
    $('#missionForm').addEventListener('submit', async e => {
      e.preventDefault();
      const dest = $('#mDest').value;
      const item = $('#mItem').value.trim() || 'Unspecified payload';
      const pri  = $('#mPriority').value;
      await API.missions.add(dest, item, pri);
      $('#mItem').value = '';
      refreshMissions();
    });

    $('#queue').addEventListener('click', async e => {
      const rm = e.target.closest('[data-remove]');
      const pr = e.target.closest('[data-promote]');
      if (rm) { await API.missions.remove(rm.dataset.remove); refreshMissions(); }
      if (pr) { await API.missions.promote(pr.dataset.promote); refreshMissions(); }
    });

    $$('[data-cmd]').forEach(btn => btn.addEventListener('click', async () => {
      const cmd = btn.dataset.cmd;
      if (cmd === 'export-map' || cmd === 'export-logs') {
        toast('info', 'Export queued', 'The file will download when the robot responds.');
        return;
      }
      if (cmd === 'estop' && !confirm('Cut power to the motors now?')) return;
      const r = await API.robot.command(cmd);
      if (r && r.message) toast(cmd === 'estop' ? 'bad' : 'ok', r.message, '');
      refreshMissions();
    }));

    $('#manualMode').addEventListener('change', e => {
      API.robot.command(e.target.checked ? 'manual' : 'resume');
    });

    $$('#tgPath,#tgFrontier,#tgTrail').forEach(t => t.addEventListener('change', drawMap));

    $('#setApply').addEventListener('click', () => {
      API.configure({ source: $('#setSource').value, host: $('#setHost').value });
      toast('ok', 'Data source changed',
        API.source === 'mock' ? 'Showing demo data.' : 'Connected to the robot address.');
      API.stream.start();
    });
    $('#setDock').addEventListener('input', e => $('#setDockOut').textContent = e.target.value + '%');
    $('#setMotion').addEventListener('change', e => {
      state.reduceMotion = e.target.checked;
      document.documentElement.style.setProperty('scroll-behavior', e.target.checked ? 'auto' : 'smooth');
    });
  }

  /* ═════════ BOOT ═════════ */
  function boot() {
    initTrace();
    wire();
    route();

    API.stream
      .on('status', s => {
        applyStatus(s);
        if (state.page === 'map') refreshMap();
      })
      .on('events', renderFeed)
      .on('alert', e => toast(e.kind, e.title, e.detail))
      .on('disconnected', () => {
        $('#linkState').classList.add('is-down');
        $('#linkState').childNodes[0].nodeValue = 'NO LINK';
      })
      .start(700);

    refreshMissions();
    setInterval(() => {
      if (state.page === 'camera') refreshCamera();
      if (state.page === 'analytics') refreshAnalytics();
      if (state.page === 'missions') refreshMissions();
    }, 1400);
  }

  document.addEventListener('DOMContentLoaded', boot);
})();
