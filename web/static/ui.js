/* Presentation, saved-map gallery and local appearance preferences. */
let savedCards = [];
let activeMapName = '';
let galleryGeneration = 0;
let galleryObjectUrls = [];
let toastTimer;
let stopHome;
let dialogReturnFocus;

function notifyUser(message, error = false) {
  clearTimeout(toastTimer);
  const toast = $('toast');
  toast.textContent = message;
  toast.classList.toggle('error', error);
  toast.hidden = false;
  toastTimer = setTimeout(() => { toast.hidden = true; }, error ? 6500 : 3500);
}

function closeDialogs() {
  document.querySelectorAll('dialog[open]').forEach(dialog => dialog.close());
  if (stopHome) stopHome.append($('quickStop'));
}

function openDialog(id) {
  if (id === 'mapsDialog') { setControlView('maps'); return; }
  stopRobot();
  closeDialogs();
  dialogReturnFocus = document.activeElement;
  const dialog = $(id);
  // There is still one STOP button: move it into the modal's visible heading.
  dialog.querySelector('.dialog-heading').insertBefore($('quickStop'), dialog.querySelector('[data-close-dialog]'));
  dialog.showModal();
}

function setDrivePanel(open) {
  setControlView(open ? 'manual' : 'nav');
}

function setControlView(view) {
  const cockpit = document.querySelector('.cockpit');
  if (cockpit.dataset.view === 'manual' && view !== 'manual') stopRobot();
  cockpit.dataset.view = view;
  document.querySelectorAll('[data-panel]').forEach(panel => { panel.hidden = panel.dataset.panel !== view; });
  Object.entries({ nav: 'viewNav', manual: 'toggleDrive', maps: 'viewMaps' }).forEach(([name, id]) => {
    $(id).classList.toggle('active', name === view);
    $(id).setAttribute('aria-selected', String(name === view));
  });
  requestAnimationFrame(resizeCanvas);
}

function applyTheme(theme, announce = true) {
  theme = theme === 'kawaii' ? 'kawaii' : 'dark';
  document.documentElement.dataset.theme = theme;
  $('brandTitle').textContent = theme === 'kawaii' ? 'Hello Kitty' : 'UGV01';
  try { localStorage.setItem('x3-theme', theme); } catch (_) {}
  $('themeSettings').hidden = theme !== 'kawaii';
  $('sanrioCredit').hidden = theme !== 'kawaii';
  document.querySelector('meta[name="theme-color"]').content = theme === 'kawaii' ? '#fff6fb' : '#171c21';
  updateMapPalette();
  rasterSource = null;
  costmapRasterSource = null;
  drawMap();
  if (announce) notifyUser(theme === 'kawaii' ? 'Hello Kitty & friends ♡' : 'Тёмный пульт');
}

function toggleTheme() {
  applyTheme(document.documentElement.dataset.theme === 'kawaii' ? 'dark' : 'kawaii');
}

function displayMapName(name) {
  return name.startsWith('before_ui_') ? 'Карта перед обновлением' : name;
}

function updateDashboard(data) {
  activeMapName = data.map || '';
  $('currentMapName').textContent = activeMapName ? displayMapName(activeMapName) : 'Живая карта';
  $('currentMapName').title = activeMapName;
  $('mapModeLabel').textContent = navigation.stack_mode === 'nav_map' ? 'Готовая карта' : 'Живая карта';
  if (!requestBusy) $('mainStackMode').value = navigation.stack_mode || 'mapping';
  $('localizationSettings').hidden = navigation.stack_mode !== 'nav_map';
  $('driveModeLabel').textContent = { JOY: 'Ручной режим', AUTO: 'Автообход', NAV: 'Навигация' }[mode] || mode;
  const canSave = !!mapMsg && ['mapping', 'nav_slam'].includes(navigation.stack_mode);
  $('openSave').disabled = $('saveFromMaps').disabled = !canSave || requestBusy;
  $('openSave').title = canSave ? 'Сохранить карту' : 'Сохранение доступно при построении карты';
  $('drivePanel').classList.toggle('auto', mode !== 'JOY');
  $('driveHint').textContent = mode === 'AUTO' ? 'Робот объезжает помещение самостоятельно' :
    mode === 'NAV' ? 'Чтобы вести вручную, нажмите «Вручную»' : 'Отпустите джойстик, чтобы остановиться';
  $('navDot').className = 'dot ' + (navigation.ready ? 'ok' : navigation.error ? 'bad' : 'warn');
  $('emptyMapText').textContent = serverOnline ? 'Ожидаем первые данные лидара' : 'Подключаемся к роботу…';
}

function renderGallery() {
  const generation = ++galleryGeneration;
  galleryObjectUrls.forEach(URL.revokeObjectURL);
  galleryObjectUrls = [];
  const gallery = $('mapGallery');
  gallery.replaceChildren();
  const search = $('mapSearch').value.trim().toLocaleLowerCase();
  const items = savedCards.filter(item => displayMapName(item.name).toLocaleLowerCase().includes(search));
  $('mapsCount').textContent = `Сохранено карт: ${savedCards.length}`;
  if (!items.length) {
    const empty = document.createElement('div');
    empty.className = 'gallery-empty';
    const title = document.createElement('h3');
    title.textContent = search ? 'Карта не найдена' : 'Пока нет сохранённых карт';
    const description = document.createElement('p');
    description.textContent = search ? 'Попробуйте другое название.' : 'Постройте карту и нажмите «Сохранить текущую».';
    empty.append(title, description); gallery.append(empty); return;
  }
  items.forEach(item => {
    const card = document.createElement('button');
    card.type = 'button';
    card.className = 'map-card' + (item.name === activeMapName ? ' current' : '');
    card.setAttribute('aria-label', `Открыть карту «${displayMapName(item.name)}»`);
    card.disabled = item.available === false || requestBusy;
    const image = document.createElement('img');
    image.alt = `Превью карты ${displayMapName(item.name)}`;
    image.width = 360; image.height = 240;
    const body = document.createElement('span'); body.className = 'map-card-body';
    const title = document.createElement('strong'); title.textContent = displayMapName(item.name); title.title = item.name;
    const details = document.createElement('span');
    const dimensions = item.resolution ? `${(item.width * item.resolution).toFixed(1)} × ${(item.height * item.resolution).toFixed(1)} м` : 'Открыть карту';
    details.textContent = item.available === false ? 'Файл карты недоступен' : dimensions;
    body.append(title, details);
    card.append(image, body);
    if (item.name === activeMapName) {
      const badge = document.createElement('span'); badge.className = 'map-card-badge'; badge.textContent = 'Открыта'; card.append(badge);
    }
    card.addEventListener('click', () => switchStackMode('nav_map', item.name));
    gallery.append(card);
    if (item.preview_url) fetch(item.preview_url, { headers: apiHeaders() }).then(response => {
      if (!response.ok) throw new Error('Превью недоступно');
      return response.blob();
    }).then(blob => {
      if (generation !== galleryGeneration || !image.isConnected) return;
      const url = URL.createObjectURL(blob); galleryObjectUrls.push(url); image.src = url;
    }).catch(() => { image.alt = 'Превью недоступно'; });
  });
}

function drawSavePreview() {
  const preview = $('savePreview');
  const size = preview.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  preview.width = Math.max(1, size.width * dpr); preview.height = Math.max(1, size.height * dpr);
  const context = preview.getContext('2d'); context.fillStyle = mapPalette.bg;
  context.fillRect(0, 0, preview.width, preview.height);
  if (!mapMsg) return;
  const raster = getMapRaster(mapMsg);
  const scale = Math.min((preview.width - 32 * dpr) / raster.width, (preview.height - 24 * dpr) / raster.height);
  context.imageSmoothingEnabled = false;
  context.drawImage(raster, (preview.width - raster.width * scale) / 2, (preview.height - raster.height * scale) / 2, raster.width * scale, raster.height * scale);
}

function showSaveDialog() {
  if (!mapMsg || !['mapping', 'nav_slam'].includes(navigation.stack_mode)) {
    notifyUser('Сохранение доступно при построении карты', true); return;
  }
  $('saveMessage').textContent = '';
  $('saveMessage').classList.remove('error');
  openDialog('saveDialog');
  requestAnimationFrame(drawSavePreview);
  $('mapName').focus();
}

async function switchStackMode(nextMode, name = '') {
  if (requestBusy) return;
  if (nextMode === navigation.stack_mode && (nextMode !== 'nav_map' || name === activeMapName)) { closeDialogs(); setControlView('nav'); return; }
  if (nextMode === 'nav_map' && !name) { setControlView('maps'); notifyUser('Выберите сохранённую карту'); await reloadMaps(); return; }
  requestBusy = true;
  $('mapsMessage').textContent = 'Открываем карту…';
  $('mainStackMode').disabled = true;
  $('navStatus').textContent = 'Переключение режима…';
  updateSelectionStatus();
  stopRobot();
  try {
    const data = await apiPost('/api/navigation/mode', { mode: nextMode, map: name });
    if (data.ok) {
      $('stackMode').value = nextMode;
      if (name) $('savedMap').value = name;
      selection = null; plannedPath = [];
      closeDialogs();
      setControlView('nav');
      requestAnimationFrame(fitMapView);
      notifyUser(nextMode === 'nav_map' ? `Открыта карта «${displayMapName(name)}»` : 'Режим включён');
    } else {
      $('mapsMessage').textContent = data.error;
      $('mapsMessage').classList.add('error');
    }
    await refreshServerStatus();
  } finally {
    requestBusy = false;
    $('mainStackMode').disabled = false;
    $('mainStackMode').value = navigation.stack_mode || 'mapping';
    updateSelectionStatus();
  }
}

function initDashboard() {
  stopHome = $('quickStop').parentElement;
  document.querySelectorAll('dialog').forEach(dialog => {
    dialog.addEventListener('close', () => {
      const current = document.querySelector('dialog[open]');
      if (current) current.querySelector('.dialog-heading').insertBefore($('quickStop'), current.querySelector('[data-close-dialog]'));
      else {
        stopHome.append($('quickStop'));
        if (dialogReturnFocus && dialogReturnFocus.isConnected) dialogReturnFocus.focus();
      }
    });
    dialog.addEventListener('click', event => {
      const rect = dialog.getBoundingClientRect();
      if (event.target === dialog && (event.clientX < rect.left || event.clientX > rect.right || event.clientY < rect.top || event.clientY > rect.bottom)) dialog.close();
    });
  });
  document.querySelectorAll('[data-close-dialog]').forEach(button => button.addEventListener('click', closeDialogs));
  const showMaps = async () => { $('mapsMessage').textContent = ''; setControlView('maps'); await reloadMaps(); };
  $('openMaps').addEventListener('click', showMaps);
  $('viewMaps').addEventListener('click', showMaps);
  $('viewNav').addEventListener('click', () => setControlView('nav'));
  $('openSettings').addEventListener('click', () => openDialog('settingsDialog'));
  $('mobileSettings').addEventListener('click', () => openDialog('settingsDialog'));
  $('openSave').addEventListener('click', showSaveDialog);
  $('saveFromMaps').addEventListener('click', showSaveDialog);
  $('mapSearch').addEventListener('input', renderGallery);
  $('newMap').addEventListener('click', () => switchStackMode('nav_slam'));
  $('toggleDrive').addEventListener('click', () => setControlView('manual'));
  $('mainStackMode').addEventListener('change', () => switchStackMode($('mainStackMode').value, $('mainStackMode').value === 'nav_map' ? activeMapName : ''));
  $('dismissMapHint').addEventListener('click', () => { $('mapHint').hidden = true; });
  $('showPath').addEventListener('change', drawMap);
  $('themeSecret').addEventListener('click', toggleTheme);
  $('themeSettings').addEventListener('click', () => applyTheme('dark'));
  let secretClicks = 0, secretTimer;
  $('brandSecret').addEventListener('click', () => {
    clearTimeout(secretTimer); secretClicks++;
    if (secretClicks >= 5) { secretClicks = 0; toggleTheme(); }
    secretTimer = setTimeout(() => { secretClicks = 0; }, 1600);
  });
  applyTheme(document.documentElement.dataset.theme, false);
  setControlView('nav');
}
