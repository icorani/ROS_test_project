const canvas = document.getElementById('c');
    const ctx = canvas.getContext('2d');
    const statusEl = document.getElementById('status');

    function resize() {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
    }

    window.addEventListener('resize', resize);
    resize();

    // ── Состояние ─────────────────────────────────────────
    const SCALE = 60;
    let camera = {x: 6, y: 4};
    let mapData = null;
    let robots = [];
    let selectedRobotId = null;
    let manualMode = false;
    let manualRobotId = null;
    let keys = {w: false, a: false, s: false, d: false};

    // ── Преобразование мир → экран ────────────────────────
    function toPx(x, y) {
        return [
            canvas.width / 2 + (x - camera.x) * SCALE,
            canvas.height / 2 - (y - camera.y) * SCALE
        ];
    }

    // ── Сетка ─────────────────────────────────────────────
    function drawGrid() {
        ctx.strokeStyle = '#2a2a2a';
        ctx.lineWidth = 1;
        const range = 30;
        const cx = Math.round(camera.x);
        const cy = Math.round(camera.y);
        for (let i = cx - range; i <= cx + range; i++) {
            const [x1, y1] = toPx(i, cy - range);
            const [x2, y2] = toPx(i, cy + range);
            ctx.beginPath();
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
        }
        for (let j = cy - range; j <= cy + range; j++) {
            const [x3, y3] = toPx(cx - range, j);
            const [x4, y4] = toPx(cx + range, j);
            ctx.beginPath();
            ctx.moveTo(x3, y3);
            ctx.lineTo(x4, y4);
            ctx.stroke();
        }
    }

    // ── Оси ───────────────────────────────────────────────
    function drawAxes() {
        ctx.strokeStyle = '#555';
        ctx.lineWidth = 1;
        const xStart = toPx(-10, 0);
        const xEnd = toPx(20, 0);
        ctx.beginPath();
        ctx.moveTo(xStart[0], xStart[1]);
        ctx.lineTo(xEnd[0], xEnd[1]);
        ctx.stroke();
        const yStart = toPx(0, -10);
        const yEnd = toPx(0, 20);
        ctx.beginPath();
        ctx.moveTo(yStart[0], yStart[1]);
        ctx.lineTo(yEnd[0], yEnd[1]);
        ctx.stroke();
        ctx.fillStyle = '#777';
        ctx.font = '11px monospace';
        ctx.textAlign = 'center';
        for (let x = -10; x <= 20; x += 2) {
            const [px, py] = toPx(x, 0);
            ctx.fillText(x.toString(), px, py + 14);
        }
        ctx.textAlign = 'right';
        for (let y = -10; y <= 20; y += 2) {
            const [px, py] = toPx(0, y);
            ctx.fillText(y.toString(), px - 6, py + 4);
        }
        ctx.fillStyle = '#aaa';
        ctx.textAlign = 'right';
        const [zeroPx, zeroPy] = toPx(0, 0);
        ctx.fillText('0', zeroPx - 6, zeroPy + 14);
        ctx.textAlign = 'left';
    }

    // ── Стены ─────────────────────────────────────────────
    function drawWalls() {
    if (!mapData) return;
    ctx.strokeStyle = '#888';
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    
    // Периметр (из bounds) — если walls нет
    if (!mapData.walls || mapData.walls.length === 0) {
        const b = mapData.bounds;
        const [x1, y1] = toPx(b.x_min, b.y_min);
        const [x2, y2] = toPx(b.x_max, b.y_min);
        const [x3, y3] = toPx(b.x_max, b.y_max);
        const [x4, y4] = toPx(b.x_min, b.y_max);
        ctx.beginPath();
        ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.lineTo(x3, y3); ctx.lineTo(x4, y4); ctx.closePath();
        ctx.stroke();
        return;
    }
    
    // Все стены (внешние + внутренние) из JSON
    mapData.walls.forEach(w => {
        const [x1, y1] = toPx(w.x1, w.y1);
        const [x2, y2] = toPx(w.x2, w.y2);
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
    });
    }

    // ── Граф ──────────────────────────────────────────────
    function drawGraph() {
        if (!mapData) return;
        const nodes = mapData.nodes;
        const edges = mapData.edges;
        ctx.strokeStyle = '#3a5a7a';
        ctx.lineWidth = 3;
        edges.forEach(([a, b]) => {
            if (!nodes[a] || !nodes[b]) return;
            const [x1, y1] = toPx(nodes[a].x, nodes[a].y);
            const [x2, y2] = toPx(nodes[b].x, nodes[b].y);
            ctx.beginPath();
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
        });
        Object.entries(nodes).forEach(([id, n]) => {
            const [px, py] = toPx(n.x, n.y);
            const isParking = id.startsWith('P');
            ctx.fillStyle = isParking ? '#a3e' : '#4af';
            ctx.beginPath();
            ctx.arc(px, py, isParking ? 6 : 5, 0, 2 * Math.PI);
            ctx.fill();
            ctx.fillStyle = '#888';
            ctx.font = '10px monospace';
            ctx.fillText(id, px + 8, py - 6);
        });
    }

    // ── Роботы ────────────────────────────────────────────
    function drawRobot(r) {
        const [px, py] = toPx(r.x, r.y);
        const color = (mapData && mapData.robots && mapData.robots[r.id])
            ? mapData.robots[r.id].color : '#4af';

        // Обводка выбранного/управляемого
        if (r.id === manualRobotId) {
            ctx.strokeStyle = '#f44';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.arc(px, py, 0.35 * SCALE, 0, 2 * Math.PI);
            ctx.stroke();
        } else if (r.id === selectedRobotId) {
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(px, py, 0.3 * SCALE, 0, 2 * Math.PI);
            ctx.stroke();
        }

        ctx.save();
        ctx.translate(px, py);
        ctx.rotate(-r.yaw);
        ctx.fillStyle = r.status === 'error' ? '#f44' : (r.status === 'moving' ? '#4f4' : (r.status === 'manual' ? '#f4a' : color));
        ctx.fillRect(-0.15 * SCALE, -0.1 * SCALE, 0.3 * SCALE, 0.2 * SCALE);
        ctx.restore();

        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(px + Math.cos(r.yaw) * 0.25 * SCALE, py - Math.sin(r.yaw) * 0.25 * SCALE);
        ctx.stroke();

        ctx.fillStyle = '#fff';
        ctx.font = '12px monospace';
        ctx.fillText(r.id, px + 15, py - 15);
    }

    // ── Таблица ───────────────────────────────────────────
    function updateRobotTable(robots) {
        const tbody = document.getElementById('robot-table-body');
        tbody.innerHTML = '';
        robots.forEach(r => {
            const tr = document.createElement('tr');
            tr.dataset.robotId = r.id;
            if (r.id === selectedRobotId) tr.classList.add('selected');

            let missionHtml = '—';
            if (r.mission && r.mission.length > 0) {
                const idx = r.mission_idx || 0;
                missionHtml = r.mission.map((node, i) => {
                    if (i < idx) return `<span class="done">${node}</span>`;
                    if (i === idx) return `<span class="current">${node}</span>`;
                    return node;
                }).join(' → ');
            }

            const isManual = r.id === manualRobotId;
            const btnText = isManual ? '■ Выход' : '▶ Управлять';
            const btnClass = isManual ? 'ctrl-btn active' : 'ctrl-btn';

            tr.innerHTML = `
      <td>${r.id}</td>
      <td>${r.x.toFixed(2)}</td>
      <td>${r.y.toFixed(2)}</td>
      <td class="status-${r.status}">${r.status}</td>
      <td class="mission">${missionHtml}</td>
      <td><button class="${btnClass}" data-robot-id="${r.id}">${btnText}</button></td>
    `;
            tbody.appendChild(tr);
        });
    }

    // ── Координаты выбранного ─────────────────────────────
    function updateCoords() {
        const el = document.getElementById('coords');
        if (!selectedRobotId) {
            el.style.display = 'none';
            return;
        }
        const r = robots.find(x => x.id === selectedRobotId);
        if (!r) {
            el.style.display = 'none';
            return;
        }
        el.style.display = 'block';
        el.classList.toggle('manual', manualMode);
        document.getElementById('coord-id').textContent = r.id + (manualMode ? ' (MANUAL)' : '');
        document.getElementById('coord-x').textContent = r.x.toFixed(2);
        document.getElementById('coord-y').textContent = r.y.toFixed(2);
        document.getElementById('coord-yaw').textContent = (r.yaw * 180 / Math.PI).toFixed(1) + '°';
    }

    // ── WebSocket ─────────────────────────────────────────
    let ws;

    function connect() {
        ws = new WebSocket(`ws://${window.location.hostname}:8191/ws`);
        ws.onopen = () => {
            statusEl.textContent = 'connected';
            statusEl.style.color = '#4f4';
        };
        ws.onmessage = (e) => {
            try {
                const msg = JSON.parse(e.data);
                if (msg.type === 'map') {
                    mapData = msg.data;
                    if (!selectedRobotId && msg.data.robots) {
                        selectedRobotId = Object.keys(msg.data.robots)[0];
                    }
                } else if (msg.type === 'state') {
                    robots = msg.robots || [];
                    updateRobotTable(robots);
                    updateCoords();
                }
            } catch (err) {
                console.error(err);
            }
        };
        ws.onerror = () => {
            statusEl.textContent = 'ws error';
            statusEl.style.color = '#f44';
        };
        ws.onclose = () => {
            statusEl.textContent = 'disconnected — reconnecting…';
            statusEl.style.color = '#f44';
            setTimeout(connect, 1000);
        };
    }

    connect();

    // ── Управление ────────────────────────────────────────
    function sendCmd() {
        if (!manualMode || !manualRobotId || !ws) return;
        let linear = 0, angular = 0;
        if (keys.w) linear += 0.3;
        if (keys.s) linear -= 0.3;
        if (keys.a) angular += 1.0;
        if (keys.d) angular -= 1.0;
        ws.send(JSON.stringify({
            type: 'cmd_vel',
            robot_id: manualRobotId,
            linear: linear,
            angular: angular
        }));
    }

    setInterval(sendCmd, 50);

    function toggleManual(robotId) {
        const send = (obj) => {
            if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
        };

        // Если уже управляем ЭТИМ роботом — выходим
        if (manualMode && manualRobotId === robotId) {
            send({type: 'cmd_vel', robot_id: robotId, linear: 0, angular: 0});
            send({type: 'resume', robot_id: robotId});
            manualMode = false;
            manualRobotId = null;
            keys = {w: false, a: false, s: false, d: false};
            updateCoords();
            return;
        }

        // Если управляем ДРУГИМ роботом — сначала выходим из него
        if (manualMode && manualRobotId && manualRobotId !== robotId) {
            send({type: 'cmd_vel', robot_id: manualRobotId, linear: 0, angular: 0});
            send({type: 'resume', robot_id: manualRobotId});
            manualMode = false;
            manualRobotId = null;
            keys = {w: false, a: false, s: false, d: false};
        }

        // Вход в ручной режим
        manualMode = true;
        manualRobotId = robotId;
        selectedRobotId = robotId;
        send({type: 'cmd_vel', robot_id: robotId, linear: 0, angular: 0});
        updateCoords();
    }

    // Клик по строке таблицы
    document.getElementById('robot-table-body').addEventListener('click', (e) => {
        const tr = e.target.closest('tr');
        if (!tr) return;
        const rid = tr.dataset.robotId;
        if (!rid) return;

        if (e.target.classList.contains('ctrl-btn')) {
            toggleManual(rid);
        } else {
            selectedRobotId = rid;
        }
        updateCoords();
    });

    // WASD
    window.addEventListener('keydown', (e) => {
    if (!manualMode) return;
    const code = e.code;
    if (code === 'KeyW') keys.w = true;
    if (code === 'KeyA') keys.a = true;
    if (code === 'KeyS') keys.s = true;
    if (code === 'KeyD') keys.d = true;
    if (code === 'Escape' && manualRobotId) {
        toggleManual(manualRobotId);
    }
    });

    window.addEventListener('keyup', (e) => {
    const code = e.code;
    if (code === 'KeyW') keys.w = false;
    if (code === 'KeyA') keys.a = false;
    if (code === 'KeyS') keys.s = false;
    if (code === 'KeyD') keys.d = false;
    });

    // ── Главный цикл ──────────────────────────────────────
    function loop() {
        // Камера следует за выбранным роботом
        if (selectedRobotId) {
            const r = robots.find(x => x.id === selectedRobotId);
            if (r) {
                camera.x += (r.x - camera.x) * 0.1;
                camera.y += (r.y - camera.y) * 0.1;
            }
        }

        ctx.fillStyle = '#1a1a1a';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        drawGrid();
        drawAxes();
        drawWalls();
        drawGraph();
        robots.forEach(drawRobot);
        requestAnimationFrame(loop);
    }

    loop();
