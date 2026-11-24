const $ = (id) => document.getElementById(id);

    const I18N = {
      ru: { title: "UGV01-X3 ROS 2 Dashboard", subtitle: "2D SLAM /map · джойстик /cmd_vel · ROS 2 Jazzy", map: "Живая /map", status: "Телеметрия", control: "Управление", connecting: "Подключение", online: "Онлайн", offline: "Офлайн" },
      en: { title: "UGV01-X3 ROS 2 Dashboard", subtitle: "2D SLAM /map · joystick /cmd_vel · ROS 2 Jazzy", map: "Live /map", status: "Telemetry", control: "Drive control", connecting: "Connecting", online: "Online", offline: "Offline" },
      ja: { title: "UGV01-X3 ROS 2 Dashboard", subtitle: "2D SLAM /map・ジョイスティック /cmd_vel・ROS 2 Jazzy", map: "ライブ /map", status: "テレメトリ", control: "走行制御", connecting: "接続中", online: "オンライン", offline: "オフライン" },
      zh: { title: "UGV01-X3 ROS 2 Dashboard", subtitle: "2D SLAM /map · 摇杆 /cmd_vel · ROS 2 Jazzy", map: "实时 /map", status: "遥测", control: "驾驶控制", connecting: "连接中", online: "在线", offline: "离线" },
      ko: { title: "UGV01-X3 ROS 2 Dashboard", subtitle: "2D SLAM /map · 조이스틱 /cmd_vel · ROS 2 Jazzy", map: "실시간 /map", status: "텔레메트리", control: "주행 제어", connecting: "연결 중", online: "온라인", offline: "오프라인" },
      es: { title: "UGV01-X3 ROS 2 Dashboard", subtitle: "2D SLAM /map · joystick /cmd_vel · ROS 2 Jazzy", map: "Mapa /map", status: "Telemetría", control: "Control", connecting: "Conectando", online: "Online", offline: "Offline" },
      pl: { title: "UGV01-X3 ROS 2 Dashboard", subtitle: "2D SLAM /map · joystick /cmd_vel · ROS 2 Jazzy", map: "Mapa /map", status: "Telemetria", control: "Sterowanie", connecting: "Łączenie", online: "Online", offline: "Offline" },
      hi: { title: "UGV01-X3 ROS 2 Dashboard", subtitle: "2D SLAM /map · joystick /cmd_vel · ROS 2 Jazzy", map: "लाइव /map", status: "Telemetry", control: "Drive control", connecting: "Connecting", online: "Online", offline: "Offline" }
    };

    const wsUrlInput = $("wsUrl");
    wsUrlInput.value = `ws://${location.hostname || "127.0.0.1"}:9090`;

    let ws = null;
    let reconnectTimer = null;
    let connected = false;
    let mode = "JOY";

    let mapMsg = null;
    let odom = { x: 0, y: 0, yaw: 0 };
    let robotPose = null;
    let robotPoseAt = 0;
    let navigation = {};
    let plannedPath = [];
    let mapTool = 'pan';
    let selection = null;
    let selecting = false;
    let serverOnline = false;

    const mapView = {
      zoom: 1,
      panX: 0,
      panY: 0,
      dragging: false,
      lastX: 0,
      lastY: 0
    };
    let target = { linear: 0, angular: 0 };
    let current = { linear: 0, angular: 0 };

    const MAX_LINEAR = 0.175;
    const MAX_ANGULAR = 0.90;
    const RATE_MS = 60;

    const joy = {
      active: false,
      nx: 0,
      ny: 0,
      radius: 105,
    };

    const keys = { w: false, a: false, s: false, d: false };

    function log(msg) {
      const el = $("log");
      const time = new Date().toLocaleTimeString();
      const line = document.createElement('div');
      line.textContent = `[${time}] ${msg}`;
      el.prepend(line);
      while (el.children.length > 80) el.lastChild.remove();
    }

    function setConn(state) {
      connected = state === "ok";
      const dot = $("connDot");
      dot.className = "dot " + (connected ? "ok" : state === "bad" ? "bad" : "warn");
      const lang = I18N[$("lang").value] || I18N.ru;
      $("connText").textContent = connected ? lang.online : state === "bad" ? lang.offline : lang.connecting;
    }

    function connect() {
      try {
        clearTimeout(reconnectTimer);
        if (ws) { ws.onclose = null; ws.onmessage = null; ws.close(); }
        setConn("warn");
        ws = new WebSocket(wsUrlInput.value.trim());

        ws.onopen = () => {
          setConn("ok");
          log("rosbridge connected");

          send({ op: "advertise", topic: "/cmd_vel_joy", type: "geometry_msgs/msg/Twist" });
          send({ op: "advertise", topic: "/ugv01/stop", type: "std_msgs/msg/Empty" });
          send({ op: "subscribe", topic: "/map", type: "nav_msgs/msg/OccupancyGrid", throttle_rate: 900,
            qos: { durability: "transient_local", reliability: "reliable", depth: 1 } });
          send({ op: "subscribe", topic: "/odom", type: "nav_msgs/msg/Odometry", throttle_rate: 120 });
          send({ op: "subscribe", topic: "/ugv01/robot_pose", type: "geometry_msgs/msg/PoseStamped", throttle_rate: 120 });
          send({ op: "subscribe", topic: "/plan", type: "nav_msgs/msg/Path", throttle_rate: 500 });
        };

        ws.onclose = () => {
          setConn("bad");
          log("rosbridge disconnected");
          stopRobot();
          robotPose = null;
          reconnectTimer = setTimeout(connect, 1400);
        };

        ws.onerror = () => {
          setConn("bad");
        };

        ws.onmessage = (event) => {
          let data;
          try { data = JSON.parse(event.data); } catch (_) { return; }
          if (data.op !== "publish") return;

          if (data.topic === '/ugv01/robot_pose') {
            const p = data.msg.pose;
            robotPose = { x: p.position.x, y: p.position.y, yaw: quatYaw(p.orientation) };
            robotPoseAt = Date.now();
            drawMap();
          }
          if (data.topic === '/plan') {
            plannedPath = data.msg.header.frame_id === 'map' ? data.msg.poses.map(p => p.pose.position) : [];
            drawMap();
          }

          if (data.topic === "/map") {
            mapMsg = data.msg;
            $("mapInfo").textContent = `map: ${mapMsg.info.width}×${mapMsg.info.height}`;
            drawMap();
          }

          if (data.topic === "/odom") {
            const p = data.msg.pose.pose.position;
            const q = data.msg.pose.pose.orientation;
            odom.x = p.x;
            odom.y = p.y;
            odom.yaw = quatYaw(q);
          }
        };
      } catch (e) {
        setConn("bad");
        log(e.message);
      }
    }

    function send(obj) {
      if (!ws || ws.readyState !== WebSocket.OPEN) return;
      ws.send(JSON.stringify(obj));
    }

    function publishCmd(linear, angular) {
      send({
        op: "publish",
        topic: "/cmd_vel_joy",
        msg: {
          linear: { x: linear, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: angular }
        }
      });
      $("cmdStat").textContent = `${linear.toFixed(2)} / ${angular.toFixed(2)}`;
    }

    function stopRobot() {
      Object.keys(keys).forEach(k => keys[k] = false);
      joy.active = false;
      target.linear = 0;
      target.angular = 0;
      current.linear = 0;
      current.angular = 0;
      joy.nx = 0;
      joy.ny = 0;
      updateJoyVisual();
      for (let i = 0; i < 4; i++) setTimeout(() => publishCmd(0, 0), i * 80);
      log("STOP");
    }

    function deadzone(v, dz = 0.08) {
      if (Math.abs(v) < dz) return 0;
      return Math.sign(v) * ((Math.abs(v) - dz) / (1 - dz));
    }

    function expo(v, power = 1.55) {
      return Math.sign(v) * Math.pow(Math.abs(v), power);
    }

    function computeTarget() {
      if (mode !== "JOY") {
        target.linear = 0;
        target.angular = 0;
        return;
      }

      let nx = joy.nx;
      let ny = joy.ny;

      if (!joy.active) {
        let kx = 0, ky = 0;
        if (keys.w) ky -= 1;
        if (keys.s) ky += 1;
        if (keys.a) kx -= 1;
        if (keys.d) kx += 1;
        nx = kx;
        ny = ky;
      }

      const throttle = expo(deadzone(-ny), 1.35);
      const steer = expo(deadzone(nx), 1.65);

      target.linear = throttle * MAX_LINEAR;
      target.angular = -steer * MAX_ANGULAR;
    }

    function driveLoop() {
      if (mode !== "JOY" || !serverOnline || document.hidden) return;
      computeTarget();

      current.linear += (target.linear - current.linear) * 0.26;
      current.angular += (target.angular - current.angular) * 0.26;

      if (Math.abs(current.linear) < 0.003) current.linear = 0;
      if (Math.abs(current.angular) < 0.01) current.angular = 0;

      publishCmd(current.linear, current.angular);
    }

    function quatYaw(q) {
      const siny = 2 * (q.w * q.z + q.x * q.y);
      const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
      return Math.atan2(siny, cosy);
    }

    const canvas = $("mapCanvas");
    const ctx = canvas.getContext("2d");

    function resizeCanvas() {
      const r = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(r.width * dpr));
      canvas.height = Math.max(1, Math.floor(r.height * dpr));
      drawMap();
    }

    function drawMap() {
      const w = canvas.width;
      const h = canvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = "#090A0F";
      ctx.fillRect(0, 0, w, h);

      ctx.strokeStyle = "rgba(255,255,255,0.035)";
      ctx.lineWidth = 1;
      const gridStep = 64 * (window.devicePixelRatio || 1);
      for (let gx = 0; gx < w; gx += gridStep) {
        ctx.beginPath();
        ctx.moveTo(gx, 0);
        ctx.lineTo(gx, h);
        ctx.stroke();
      }
      for (let gy = 0; gy < h; gy += gridStep) {
        ctx.beginPath();
        ctx.moveTo(0, gy);
        ctx.lineTo(w, gy);
        ctx.stroke();
      }

      if (!mapMsg) {
        ctx.fillStyle = "#64748B";
        ctx.font = `${14 * (window.devicePixelRatio || 1)}px ${getComputedStyle(document.body).fontFamily}`;
        ctx.fillText("waiting for /map", 24, 36);
        return;
      }

      const info = mapMsg.info;
      const mw = info.width;
      const mh = info.height;
      const data = mapMsg.data;

      const fitScale = Math.min(w / mw, h / mh);
      const scale = fitScale * mapView.zoom;
      const ox = (w - mw * scale) / 2 + mapView.panX;
      const oy = (h - mh * scale) / 2 + mapView.panY;

      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(getMapRaster(mapMsg), ox, oy, mw * scale, mh * scale);

      const hasKnown = Array.prototype.some.call(data, v => v !== -1 && v !== 255);
      if (!hasKnown) {
        ctx.fillStyle = "rgba(226,232,240,0.80)";
        ctx.font = `${13 * (window.devicePixelRatio || 1)}px ${getComputedStyle(document.body).fontFamily}`;
        ctx.fillText("EMPTY MAP: move robot or wait for SLAM", 24, 36);
      }

      drawNavigation(info, scale, ox, oy, mh);
      if (robotPose && Date.now() - robotPoseAt < 2200) drawRobotOnMap(info, scale, ox, oy, mh);
    }

    function drawRobotOnMap(info, scale, ox, oy, mh) {
      const {x: px, y: py} = worldToCanvas(robotPose.x, robotPose.y);

      if (!Number.isFinite(px) || !Number.isFinite(py)) return;

      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-(robotPose.yaw - quatYaw(info.origin.orientation)));

      ctx.strokeStyle = "#090A0F";
      ctx.lineWidth = Math.max(2, 3 * (window.devicePixelRatio || 1));

      ctx.fillStyle = "#F8FAFC";
      ctx.strokeStyle = "#020617";
      ctx.lineWidth = Math.max(3, 4 * (window.devicePixelRatio || 1));

      ctx.beginPath();
      ctx.moveTo(34, 0);
      ctx.lineTo(-20, -18);
      ctx.lineTo(-12, 0);
      ctx.lineTo(-20, 18);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = "#020617";
      ctx.beginPath();
      ctx.arc(0, 0, 5 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
      ctx.fill();

      ctx.restore();
    }


    function updateJoyFromEvent(e) {
      const zone = $("joyZone");
      const rect = zone.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top + rect.height / 2;
      const dx = e.clientX - cx;
      const dy = e.clientY - cy;

      const maxR = Math.min(rect.width, rect.height) * 0.34;
      const dist = Math.hypot(dx, dy);
      const clamped = Math.min(maxR, dist);
      const angle = Math.atan2(dy, dx);

      const x = Math.cos(angle) * clamped;
      const y = Math.sin(angle) * clamped;

      joy.nx = x / maxR;
      joy.ny = y / maxR;
      joy.radius = maxR;

      updateJoyVisual();
    }

    function updateJoyVisual() {
      const knob = $("joyKnob");
      const vec = $("joyVector");
      const x = joy.nx * joy.radius;
      const y = joy.ny * joy.radius;

      knob.style.transform = `translate(${x}px, ${y}px)`;

      const len = Math.hypot(x, y);
      const angle = Math.atan2(y, x) + Math.PI / 2;
      vec.style.height = `${len}px`;
      vec.style.transform = `translate(-50%, -100%) rotate(${angle}rad)`;
    }

    $("joyZone").addEventListener("pointerdown", (e) => {
      if (mode !== "JOY" || !serverOnline) return;
      joy.active = true;
      $("joyZone").setPointerCapture(e.pointerId);
      updateJoyFromEvent(e);
    });

    $("joyZone").addEventListener("pointermove", (e) => {
      if (!joy.active) return;
      updateJoyFromEvent(e);
    });

    function releaseJoy() {
      joy.active = false;
      joy.nx = 0;
      joy.ny = 0;
      updateJoyVisual();
      stopRobot();
    }

    $("joyZone").addEventListener("pointerup", releaseJoy);
    $("joyZone").addEventListener("pointercancel", releaseJoy);
    window.addEventListener("blur", stopRobot);

    function setKeyVisual() {
      $("keyW").classList.toggle("active", keys.w);
      $("keyA").classList.toggle("active", keys.a);
      $("keyS").classList.toggle("active", keys.s);
      $("keyD").classList.toggle("active", keys.d);
    }

    window.addEventListener("keydown", (e) => {
      if (e.target.closest("input,select,textarea")) return;
      const k = e.key.toLowerCase();
      if (k in keys && mode === "JOY") {
        keys[k] = true;
        setKeyVisual();
        e.preventDefault();
      }
      if (k === " ") {
        emergencyStop();
        e.preventDefault();
      }
    });

    window.addEventListener("keyup", (e) => {
      const k = e.key.toLowerCase();
      if (k in keys) {
        keys[k] = false;
        setKeyVisual();
        e.preventDefault();
      }
    });


    $("lang").addEventListener("change", () => {
      const t = I18N[$("lang").value] || I18N.ru;
      $("t-title").textContent = t.title;
      $("t-subtitle").textContent = t.subtitle;
      $("t-map").textContent = t.map;
      $("t-status").textContent = t.status;
      $("t-control").textContent = t.control;
      $("connText").textContent = connected ? t.online : t.connecting;
    });

    wsUrlInput.addEventListener("change", connect);


    function zoomMapAt(factor, cx = canvas.width / 2, cy = canvas.height / 2) {
      const oldZoom = mapView.zoom;
      const newZoom = Math.min(16, Math.max(0.35, oldZoom * factor));
      const k = newZoom / oldZoom;

      mapView.panX = (cx - canvas.width / 2) * (1 - k) + mapView.panX * k;
      mapView.panY = (cy - canvas.height / 2) * (1 - k) + mapView.panY * k;
      mapView.zoom = newZoom;

      drawMap();
    }

    function fitMapView() {
      mapView.zoom = 1;
      mapView.panX = 0;
      mapView.panY = 0;
      drawMap();
      log("map view fit");
    }

    if ($("zoomIn")) $("zoomIn").addEventListener("click", () => zoomMapAt(1.25));
    if ($("zoomOut")) $("zoomOut").addEventListener("click", () => zoomMapAt(0.8));
    if ($("zoomFit")) $("zoomFit").addEventListener("click", fitMapView);

    window.addEventListener("resize", resizeCanvas);
    initNavigation();
    initMapEvents();
    resizeCanvas();
    connect();
    setInterval(driveLoop, RATE_MS);
    setInterval(refreshServerStatus, 1200);
  
