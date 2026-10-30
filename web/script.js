const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
const darkShopBtn = document.getElementById('darkShopBtn');

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
let blockedNodes = new Set();
let blockedEdges = new Set();
let reservedNodes = new Set();
let reservedEdges = new Set();
let occupiedNodes = new Set();
let darkShopEnabled = false;
let stuckRobots = new Set();

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
}

// ── Стены ─────────────────────────────────────────────
function drawWalls() {
    if (!mapData) return;
    ctx.strokeStyle = '#888';
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    
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
    
    // Draw edges
    ctx.lineWidth = 3;
    edges.forEach(([a, b]) => {
        if (!nodes[a] || !nodes[b]) return;
        const edgeKey = [a,b].sort().join(',');
        const isBlocked = blockedEdges.has(edgeKey) || 
            Array.from(blockedEdges).some(e => (e[0]===a&&e[1]===b)||(e[0]===b&&e[1]===a));
        const isReserved = reservedEdges.has(edgeKey) || 
            Array.from(reservedEdges).some(e => (e[0]===a&&e[1]===b)||(e[0]===b&&e[1]===a));
        if (isBlocked) {
            ctx.strokeStyle = '#f44';
            ctx.setLineDash([5, 5]);
        } else if (isReserved) {
            ctx.strokeStyle = '#ffa500';
            ctx.setLineDash([3, 3]);
        } else {
            ctx.strokeStyle = '#3a5a7a';
            ctx.setLineDash([]);
        }
        const [x1, y1] = toPx(nodes[a].x, nodes[a].y);
        const [x2, y2] = toPx(nodes[b].x, nodes[b].y);
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
    });
    ctx.setLineDash([]);
    
    // Draw nodes
    Object.entries(nodes).forEach(([id, n]) => {
        const [px, py] = toPx(n.x, n.y);
        const isParking = id.startsWith('P');
        const isBlocked = blockedNodes.has(id);
        const isReserved = reservedNodes.has(id);
        const isOccupied = occupiedNodes.has(id);
        if (isBlocked) {
            ctx.fillStyle = '#f44';
        } else if (isOccupied) {
            ctx.fillStyle = '#ffd700';
        } else if (isReserved) {
            ctx.fillStyle = '#ffa500';
        } else if (isParking) {
            ctx.fillStyle = '#a3e';
        } else {
            ctx.fillStyle = '#4af';
        }
        ctx.beginPath();
        ctx.arc(px, py, isParking ? 6 : 5, 0, 2 * Math.PI);
        ctx.fill();
        if (isBlocked) {
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 2;
            ctx.stroke();
        } else if (isOccupied) {
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 2;
            ctx.stroke();
        } else if (isReserved) {
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 1;
            ctx.stroke();
        }
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
    let fillColor = color;
    if (r.status === 'error' || r.status === 'disconnected') fillColor = '#f44';
    else if (r.status === 'moving') fillColor = '#4f4';
    else if (r.status === 'manual') fillColor = '#f4a';
    else if (r.status === 'done') fillColor = '#0ff';
    else if (r.status === 'inactive') fillColor = '#666';
    ctx.fillStyle = fillColor;
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
function getTargetOptions() {
    if (!mapData || !mapData.nodes) return [];
    const keys = Object.keys(mapData.nodes);
    keys.sort();
    return keys;
}

let lastRobotIds = '';
function updateRobotTable(robotsList) {
    const robotIds = (robotsList.map(r => r.id)).join(',');
    if (robotIds !== lastRobotIds) {
        const tbody = document.getElementById('robot-table-body');
        tbody.innerHTML = '';
        const targets = getTargetOptions();
        robotsList.forEach(r => {
            const tr = document.createElement('tr');
            tr.dataset.robotId = r.id;
            if (r.id === selectedRobotId) tr.classList.add('selected');
            if (!r.is_active) tr.classList.add('inactive');

            const isManual = r.id === manualRobotId;
            const btnText = isManual ? '■ Выход' : '▶ Управлять';
            const btnClass = isManual ? 'ctrl-btn active' : 'ctrl-btn';

            let orderHtml = '';
            let actionHtml = '';
            if (r.is_active) {
                orderHtml = `
          <td><input type="text" class="target-input" name="target_${r.id}" placeholder="узел/парковка" style="width:70px;"></td>
          <td><button type="button" class="order-send-btn ctrl-btn">Send</button></td>
          <td><button type="button" class="go-home-btn ctrl-btn">Домой</button></td>
          <td><button type="button" class="${btnClass}">${btnText}</button></td>
        `;
            } else {
                orderHtml = `<td colspan="4">— (неактивен)</td>`;
            }

            tr.innerHTML = `
          <td class="col-id">${r.id}</td>
          <td class="col-x">${r.x.toFixed(2)}</td>
          <td class="col-y">${r.y.toFixed(2)}</td>
          <td class="col-status status-${r.status}">${r.status}</td>
          <td class="col-mission">—</td>
          <td class="col-error">—</td>
          ${orderHtml}
        `;
            tbody.appendChild(tr);
        });
        lastRobotIds = robotIds;
    }
    // Update values
    const tbody = document.getElementById('robot-table-body');
    robotsList.forEach(r => {
        const tr = tbody.querySelector(`tr[data-robot-id="${r.id}"]`);
        if (!tr) return;
        if (r.id === selectedRobotId) tr.classList.add('selected'); else tr.classList.remove('selected');
        if (!r.is_active) tr.classList.add('inactive'); else tr.classList.remove('inactive');
        tr.querySelector('.col-x').textContent = r.x.toFixed(2);
        tr.querySelector('.col-y').textContent = r.y.toFixed(2);
        const st = tr.querySelector('.col-status');
        st.textContent = r.status;
        st.className = `col-status status-${r.status}`;
        let missionHtml = '—';
        if (r.mission && r.mission.length > 0) {
            const idx = r.mission_idx || 0;
            missionHtml = r.mission.map((node, i) => {
                if (i < idx) return `<span class="done">${node}</span>`;
                if (i === idx) return `<span class="current">${node}</span>`;
                return node;
            }).join(' → ');
        }
        tr.querySelector('.col-mission').innerHTML = missionHtml;
        const errText = r.error ? r.error : (r.status === 'disconnected' ? 'connection_lost' : '—');
        tr.querySelector('.col-error').textContent = errText;
        if (r.is_active) {
            const manBtn = Array.from(tr.querySelectorAll('button.ctrl-btn')).find(b => !b.classList.contains('order-send-btn') && !b.classList.contains('go-home-btn'));
            if (manBtn) {
                const isManual = r.id === manualRobotId;
                manBtn.textContent = isManual ? '■ Выход' : '▶ Управлять';
                manBtn.className = isManual ? 'ctrl-btn active' : 'ctrl-btn';
            }
        }
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
                blockedNodes = new Set(msg.data.blocked_nodes || []);
                blockedEdges = new Set((msg.data.blocked_edges || []).map(e => e.sort().join(',')));
                reservedNodes = new Set(Object.keys(msg.data.reserved_nodes || {}));
                reservedEdges = new Set(Object.keys(msg.data.reserved_edges || {}));
                occupiedNodes = new Set(Object.keys(msg.data.occupied_nodes || {}));
                darkShopEnabled = msg.data.dark_shop || false;
                stuckRobots = new Set(msg.data.stuck_robots || []);
                updateDarkShopBtn();
                if (!selectedRobotId && msg.data.robots) {
                    selectedRobotId = Object.keys(msg.data.robots)[0];
                }
            } else if (msg.type === 'state') {
                robots = msg.robots || [];
                blockedNodes = new Set(msg.blocked_nodes || []);
                blockedEdges = new Set((msg.blocked_edges || []).map(e => e.sort().join(',')));
                reservedNodes = new Set(Object.keys(msg.reserved_nodes || {}));
                reservedEdges = new Set(Object.keys(msg.reserved_edges || {}));
                occupiedNodes = new Set(Object.keys(msg.occupied_nodes || {}));
                darkShopEnabled = msg.dark_shop || false;
                stuckRobots = new Set(msg.stuck_robots || []);
                updateDarkShopBtn();
                updateRobotTable(robots);
                updateCoords();
            } else if (msg.type === 'dark_shop_status') {
                darkShopEnabled = msg.enabled;
                updateDarkShopBtn();
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

// ── Dark Shop Button ──────────────────────────────────
function updateDarkShopBtn() {
    darkShopBtn.textContent = darkShopEnabled ? 'Тёмный цех: ON' : 'Тёмный цех: OFF';
    darkShopBtn.classList.toggle('active', darkShopEnabled);
}

darkShopBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({type: 'dark_shop', enabled: !darkShopEnabled}));
    }
});

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

    if (manualMode && manualRobotId === robotId) {
        send({type: 'cmd_vel', robot_id: robotId, linear: 0, angular: 0});
        send({type: 'resume', robot_id: robotId});
        manualMode = false;
        manualRobotId = null;
        keys = {w: false, a: false, s: false, d: false};
        updateCoords();
        return;
    }

    if (manualMode && manualRobotId && manualRobotId !== robotId) {
        send({type: 'cmd_vel', robot_id: manualRobotId, linear: 0, angular: 0});
        send({type: 'resume', robot_id: manualRobotId});
        manualMode = false;
        manualRobotId = null;
        keys = {w: false, a: false, s: false, d: false};
    }

    manualMode = true;
    manualRobotId = robotId;
    selectedRobotId = robotId;
    send({type: 'cmd_vel', robot_id: robotId, linear: 0, angular: 0});
    updateCoords();
}

// Клик по строке таблицы
document.getElementById('robot-table-body').addEventListener('mousedown', (e) => {
    if (e.target.classList.contains('target-input')) {
        e.stopPropagation();
    }
}, true);

document.getElementById('robot-table-body').addEventListener('click', (e) => {
    if (e.target.classList.contains('target-input')) {
        return;
    }
    const tr = e.target.closest('tr');
    if (!tr) return;
    const rid = tr.dataset.robotId;
    if (!rid) return;

    if (e.target.classList.contains('order-send-btn')) {
        const inp = tr.querySelector('.target-input');
        const target = inp ? inp.value.trim() : null;
        if (target) {
            fetch(`http://${window.location.hostname}:8191/order`, {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({robot_id: rid, target: target})
            }).then(r => r.json()).then(data => {
                console.log('order result', data);
                if (data && data.ok === false && data.error === 'robot_stuck_blocked') {
                    alert('Робот заблокирован как застрявший. Нажми "Управлять"->"Выход" или дождись сброса.');
                }
            }).catch(err => console.error(err));
        }
        return;
    }

    if (e.target.classList.contains('go-home-btn')) {
        fetch(`http://${window.location.hostname}:8191/go_home`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({robot_id: rid})
        }).then(r => r.json()).then(data => {
            console.log('go_home result', data);
        }).catch(err => console.error(err));
        return;
    }

    if (e.target.classList.contains('ctrl-btn')) {
        toggleManual(rid);
        return;
    }
    selectedRobotId = rid;
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
