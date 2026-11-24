/* Map coordinates, touch selection and Nav2 HTTP controls. No third-party assets. */
let rasterSource = null;
let rasterCanvas = null;
let lastStackMode = null;
let requestBusy = false;
let costmapRasterSource = null;
let costmapRaster = null;

function apiHeaders() {
  return { 'Content-Type': 'application/json', 'X-Dashboard-Token': $('dashboardToken').value };
}

async function apiPost(path, body = {}) {
  try {
    const response = await fetch(path, { method: 'POST', headers: apiHeaders(), body: JSON.stringify(body) });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
    log(`${path}: OK`);
    return data;
  } catch (e) {
    log(e.message);
    $('navStatus').textContent = e.message;
    return { ok: false, error: e.message };
  }
}

async function emergencyStop() {
  stopRobot();
  // Immediate ROS STOP as well as HTTP action cancellation, even during mode switching.
  send({ op: 'publish', topic: '/ugv01/stop', msg: {} });
  await apiPost('/api/stop');
  mode = 'JOY';
  renderDriveMode();
}

function renderDriveMode() {
  $('modeStat').textContent = mode;
  $('joyBtn').classList.toggle('active', mode === 'JOY');
  $('autoBtn').classList.toggle('active', mode === 'AUTO');
  $('joyZone').style.opacity = mode === 'JOY' ? '1' : '.35';
}

async function refreshServerStatus() {
  try {
    const response = await fetch('/api/status', { headers: apiHeaders() });
    if (!response.ok) throw new Error(response.status === 401 ? 'Введите токен панели' : 'Сервер недоступен');
    const data = await response.json();
    serverOnline = true;
    navigation = data.navigation || {};
    if (mode !== data.mode) stopRobot();
    mode = data.mode || 'JOY';
    renderDriveMode();
    if (Date.now() - driveCmdAt > 1500) $('cmdStat').textContent = '— / —';
    const sourceChanged = lastStackMode !== navigation.stack_mode;
    if (sourceChanged) {
      if (lastStackMode !== null) {
        selection = null;
        plannedPath = [];
        costmapMsg = null;
        robotPose = null;
        if (!(lastStackMode === 'mapping' && navigation.stack_mode === 'nav_slam') &&
            !(lastStackMode === 'nav_slam' && navigation.stack_mode === 'mapping')) mapMsg = null;
      }
      lastStackMode = navigation.stack_mode;
      if (['mapping', 'nav_slam', 'nav_map'].includes(lastStackMode)) $('stackMode').value = lastStackMode;
    }
    if (navigation.pose) {
      robotPose = navigation.pose;
      robotPoseAt = Date.now();
      $('poseStat').textContent = `${robotPose.x.toFixed(2)} ${robotPose.y.toFixed(2)}`;
      $('yawStat').textContent = `${(robotPose.yaw * 180 / Math.PI).toFixed(0)}°`;
    } else {
      robotPose = null;
      $('poseStat').textContent = 'TF: ожидание';
    }
    const stateNames = { idle: 'ожидание цели', pending: 'отправка', navigating: 'движение',
      succeeded: 'цель достигнута', canceled: 'отменено', failed: 'ошибка', rejected: 'цель отклонена' };
    let message = navigation.ready ? `Nav2 готов · ${stateNames[navigation.goal_state] || navigation.goal_state}` : navigation.reason;
    if (navigation.distance_remaining != null) message += ` · ${navigation.distance_remaining.toFixed(2)} м`;
    if (navigation.error || data.error) message += ` · ${navigation.error || data.error}`;
    $('navStatus').textContent = message || 'SLAM';
    $('navStatus').classList.toggle('ready', !!navigation.ready);
    $('toolGoal').disabled = !navigation.ready;
    $('toolPose').disabled = navigation.stack_mode !== 'nav_map' || !navigation.lifecycle?.amcl;
    $('globalPose').disabled = $('toolPose').disabled;
    if (!data.managed) $('navStatus').textContent += ' · Для Nav2 запустите web_navigation.launch.py';
    updateSelectionStatus();
    drawMap();
  } catch (e) {
    if (serverOnline) stopRobot();
    serverOnline = false;
    robotPose = null;
    $('navStatus').textContent = e.message;
    $('navStatus').classList.remove('ready');
    $('sendSelection').disabled = true;
  }
}

async function reloadMaps() {
  try {
    const response = await fetch('/api/maps', { headers: apiHeaders() });
    const data = await response.json();
    if (!response.ok || !data.ok) throw new Error(data.error || 'Не удалось получить список карт');
    const selected = $('savedMap').value;
    $('savedMap').replaceChildren();
    if (!data.maps.length) $('savedMap').add(new Option('Нет сохранённых карт', ''));
    data.maps.forEach(name => $('savedMap').add(new Option(name, name)));
    if (data.maps.includes(selected)) $('savedMap').value = selected;
  } catch (e) { log(e.message); }
}

function setMapTool(tool) {
  mapTool = tool;
  selection = null;
  selecting = false;
  mapView.dragging = false;
  ['pan', 'goal', 'pose'].forEach(name => $('tool' + name[0].toUpperCase() + name.slice(1)).classList.toggle('active', tool === name));
  canvas.style.cursor = tool === 'pan' ? 'grab' : 'crosshair';
  updateSelectionStatus();
  drawMap();
}

function updateSelectionStatus() {
  $('selectionStatus').textContent = selection ?
    `${selection.tool === 'goal' ? 'Цель' : 'Поза'}: x=${selection.x.toFixed(2)} м, y=${selection.y.toFixed(2)} м, угол=${(selection.yaw * 180 / Math.PI).toFixed(0)}°` : 'Точка не выбрана';
  $('sendSelection').disabled = requestBusy || !serverOnline || !selection ||
    (selection.tool === 'goal' ? !navigation.ready : navigation.stack_mode !== 'nav_map');
}

function initNavigation() {
  $('dashboardToken').value = sessionStorage.getItem('ugvToken') || '';
  $('dashboardToken').addEventListener('change', () => {
    sessionStorage.setItem('ugvToken', $('dashboardToken').value);
    reloadMaps(); refreshServerStatus();
  });
  $('stopBtn').addEventListener('click', emergencyStop);
  $('quickStop').addEventListener('click', emergencyStop);
  for (const name of ['joy', 'auto']) $('' + name + 'Btn').addEventListener('click', async () => {
    stopRobot();
    const result = await apiPost('/api/mode/' + name);
    if (result.ok) { mode = name.toUpperCase(); renderDriveMode(); }
  });
  $('applyStack').addEventListener('click', async () => {
    if (requestBusy) return;
    requestBusy = true;
    $('applyStack').disabled = true;
    stopRobot();
    $('navStatus').textContent = 'Переключение режима…';
    try {
      await apiPost('/api/navigation/mode', { mode: $('stackMode').value, map: $('savedMap').value });
      await refreshServerStatus();
    } finally { requestBusy = false; $('applyStack').disabled = false; }
  });
  $('saveMap').addEventListener('click', async () => {
    $('saveMap').disabled = true;
    try {
      const data = await apiPost('/api/maps/save', { name: $('mapName').value.trim() });
      if (data.ok) { await reloadMaps(); $('savedMap').value = data.map; }
    } finally { $('saveMap').disabled = false; }
  });
  $('reloadMaps').addEventListener('click', reloadMaps);
  $('showCostmap').addEventListener('change', drawMap);
  $('toolPan').addEventListener('click', () => setMapTool('pan'));
  $('toolGoal').addEventListener('click', () => setMapTool('goal'));
  $('toolPose').addEventListener('click', () => setMapTool('pose'));
  $('clearSelection').addEventListener('click', () => { selection = null; updateSelectionStatus(); drawMap(); });
  $('cancelGoal').addEventListener('click', async () => {
    stopRobot(); await apiPost('/api/navigation/cancel'); plannedPath = []; await refreshServerStatus();
  });
  $('globalPose').addEventListener('click', async () => {
    stopRobot(); await apiPost('/api/localization/global'); await refreshServerStatus();
  });
  $('sendSelection').addEventListener('click', async () => {
    if (!selection || requestBusy) return;
    requestBusy = true;
    updateSelectionStatus();
    const point = { x: selection.x, y: selection.y, yaw: selection.yaw };
    const path = selection.tool === 'goal' ? '/api/navigation/goal' : '/api/localization/initialpose';
    stopRobot();
    try {
      const data = await apiPost(path, point);
      if (data.ok) { selection = null; setMapTool('pan'); }
      await refreshServerStatus();
    } finally { requestBusy = false; updateSelectionStatus(); }
  });
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopRobot(); });
  document.addEventListener('focusin', e => { if (e.target.matches('input,select,textarea')) stopRobot(); });
  reloadMaps(); refreshServerStatus();
}

function mapGeometry() {
  if (!mapMsg) return null;
  const info = mapMsg.info;
  const scale = Math.min(canvas.width / info.width, canvas.height / info.height) * mapView.zoom;
  return { info, scale, ox: (canvas.width - info.width * scale) / 2 + mapView.panX,
    oy: (canvas.height - info.height * scale) / 2 + mapView.panY };
}

function worldToCanvas(x, y) {
  const { info, scale, ox, oy } = mapGeometry();
  const a = quatYaw(info.origin.orientation);
  const dx = x - info.origin.position.x, dy = y - info.origin.position.y;
  const gx = (Math.cos(a) * dx + Math.sin(a) * dy) / info.resolution;
  const gy = (-Math.sin(a) * dx + Math.cos(a) * dy) / info.resolution;
  return { x: ox + gx * scale, y: oy + (info.height - gy) * scale };
}

function canvasToWorld(px, py) {
  const g = mapGeometry();
  if (!g) return null;
  const gx = (px - g.ox) / g.scale, gy = g.info.height - (py - g.oy) / g.scale;
  if (gx < 0 || gy < 0 || gx >= g.info.width || gy >= g.info.height) return null;
  const a = quatYaw(g.info.origin.orientation), x = gx * g.info.resolution, y = gy * g.info.resolution;
  return { x: g.info.origin.position.x + Math.cos(a) * x - Math.sin(a) * y,
    y: g.info.origin.position.y + Math.sin(a) * x + Math.cos(a) * y };
}

function eventPixel(e) {
  const rect = canvas.getBoundingClientRect();
  return { x: (e.clientX - rect.left) * canvas.width / rect.width,
    y: (e.clientY - rect.top) * canvas.height / rect.height };
}

function initMapEvents() {
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const point = eventPixel(e);
    zoomMapAt(e.deltaY < 0 ? 1.14 : 0.88, point.x, point.y);
  }, { passive: false });
  let activePointer = null;
  canvas.addEventListener('pointerdown', e => {
    if (activePointer !== null) return;
    activePointer = e.pointerId;
    canvas.setPointerCapture(e.pointerId);
    const point = eventPixel(e);
    if (mapTool === 'pan') {
      mapView.dragging = true;
      mapView.lastX = point.x; mapView.lastY = point.y;
      canvas.style.cursor = 'grabbing';
    } else {
      const world = canvasToWorld(point.x, point.y);
      if (world) {
        selection = { ...world, yaw: robotPose?.yaw || 0, tool: mapTool };
        selecting = true;
        updateSelectionStatus(); drawMap();
      }
    }
  });
  canvas.addEventListener('pointermove', e => {
    if (e.pointerId !== activePointer) return;
    const point = eventPixel(e);
    if (mapView.dragging) {
      mapView.panX += point.x - mapView.lastX; mapView.panY += point.y - mapView.lastY;
      mapView.lastX = point.x; mapView.lastY = point.y;
    } else if (selecting && selection) {
      const world = canvasToWorld(point.x, point.y);
      if (world && Math.hypot(world.x - selection.x, world.y - selection.y) > .03)
        selection.yaw = Math.atan2(world.y - selection.y, world.x - selection.x);
      updateSelectionStatus();
    }
    drawMap();
  });
  for (const event of ['pointerup', 'pointercancel', 'lostpointercapture']) canvas.addEventListener(event, e => {
    if (e.pointerId !== activePointer) return;
    if (event === 'pointercancel') selection = null;
    activePointer = null; selecting = false; mapView.dragging = false;
    canvas.style.cursor = mapTool === 'pan' ? 'grab' : 'crosshair';
    updateSelectionStatus(); drawMap();
  });
  canvas.addEventListener('dblclick', () => { if (mapTool === 'pan') fitMapView(); });
}

function getMapRaster(msg) {
  if (rasterSource === msg) return rasterCanvas;
  const { width, height } = msg.info;
  rasterCanvas = document.createElement('canvas');
  rasterCanvas.width = width; rasterCanvas.height = height;
  const context = rasterCanvas.getContext('2d');
  const pixels = context.createImageData(width, height);
  for (let y = 0; y < height; y++) for (let x = 0; x < width; x++) {
    const value = msg.data[y * width + x];
    const color = value === -1 || value === 255 ? [13, 17, 23] : value >= 65 ? [248, 250, 252] : value <= 20 ? [75, 85, 99] : [156, 163, 175];
    const index = ((height - y - 1) * width + x) * 4;
    pixels.data.set([...color, 255], index);
  }
  context.putImageData(pixels, 0, 0);
  rasterSource = msg;
  return rasterCanvas;
}

function drawCostmap() {
  if (!costmapMsg || !$('showCostmap').checked || !navigation.stack_mode?.startsWith('nav_')) return;
  const msg = costmapMsg, info = msg.info;
  if (costmapRasterSource !== msg) {
    costmapRaster = document.createElement('canvas');
    costmapRaster.width = info.width; costmapRaster.height = info.height;
    const context = costmapRaster.getContext('2d');
    const pixels = context.createImageData(info.width, info.height);
    for (let y = 0; y < info.height; y++) for (let x = 0; x < info.width; x++) {
      const value = msg.data[y * info.width + x];
      if (value <= 0 || value === 255) continue;
      const color = value >= 100 ? [255, 77, 109, 180] :
        value >= 99 ? [245, 158, 11, 145] : [252, 211, 77, Math.round(value * .8)];
      pixels.data.set(color, ((info.height - y - 1) * info.width + x) * 4);
    }
    context.putImageData(pixels, 0, 0);
    costmapRasterSource = msg;
  }
  const a = quatYaw(info.origin.orientation), c = Math.cos(a), s = Math.sin(a);
  const width = info.width * info.resolution, height = info.height * info.resolution;
  const origin = info.origin.position;
  const tl = worldToCanvas(origin.x - s * height, origin.y + c * height);
  const tr = worldToCanvas(origin.x + c * width - s * height, origin.y + s * width + c * height);
  const bl = worldToCanvas(origin.x, origin.y);
  ctx.save();
  ctx.setTransform((tr.x - tl.x) / info.width, (tr.y - tl.y) / info.width,
    (bl.x - tl.x) / info.height, (bl.y - tl.y) / info.height, tl.x, tl.y);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(costmapRaster, 0, 0);
  ctx.restore();
}

function drawNavigation(info) {
  ctx.save();
  const dpr = window.devicePixelRatio || 1;
  if (plannedPath.length && navigation.goal_state === 'navigating') {
    ctx.strokeStyle = '#22D3EE'; ctx.lineWidth = 2 * dpr; ctx.beginPath();
    plannedPath.forEach((point, index) => {
      const p = worldToCanvas(point.x, point.y);
      if (index) ctx.lineTo(p.x, p.y); else ctx.moveTo(p.x, p.y);
    });
    ctx.stroke();
  }
  function arrow(point, color) {
    const p = worldToCanvas(point.x, point.y);
    ctx.save(); ctx.translate(p.x, p.y);
    ctx.rotate(-(point.yaw - quatYaw(info.origin.orientation)));
    ctx.strokeStyle = color; ctx.lineWidth = 3 * dpr;
    ctx.beginPath(); ctx.arc(0, 0, 7 * dpr, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(32 * dpr, 0);
    ctx.lineTo(23 * dpr, -6 * dpr); ctx.moveTo(32 * dpr, 0); ctx.lineTo(23 * dpr, 6 * dpr); ctx.stroke();
    ctx.restore();
  }
  if (navigation.goal && ['pending', 'navigating', 'succeeded'].includes(navigation.goal_state)) arrow(navigation.goal, '#86EFAC');
  if (selection) arrow(selection, selection.tool === 'pose' ? '#FCD34D' : '#22D3EE');
  ctx.restore();
}
