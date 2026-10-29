# ROS 2 Warehouse Simulator

Симулятор склада с двумя автономными роботами, оркестратором и веб-визуализацией. Роботы получают приказы на перемещение, объезжают препятствия и управляются вручную через браузер.

## Стек

* **ROS 2 Jazzy** — шина связи между нодами
* **Gazebo Harmonic** — физическая симуляция (headless)
* **Python 3.12** — оркестратор, драйверы роботов
* **FastAPI** — HTTP/WebSocket API оркестратора
* **HTML5 Canvas** — визуализация склада и роботов
* **Docker Compose** — оркестрация контейнеров

---

## Быстрый старт

### 1. Клонировать репозиторий

```bash
git clone <repo-url>
cd ROS_test_project
```

### 2. Запустить контейнеры и ноды

```bash
# Поднять все контейнеры
docker compose up --build
```

> **Примечание:** Первый запуск займёт 5–10 минут (скачивание образов `ros:jazzy`, `osrf/ros:jazzy-desktop-full`, `nginx:alpine`).

В отдельных терминалах запустите ноды роботов (по одной на робота):

```bash
# Робот 1
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash && ROBOT_ID=robot_1 MAP=simple python3 /ros_ws/src/robot_sim/robot_sim/robot_driver.py"

# Робот 2
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash && ROBOT_ID=robot_2 MAP=simple python3 /ros_ws/src/robot_sim/robot_sim/robot_driver.py"
```

### 3. Открыть Canvas и API

| Сервис | Описание | Ссылка / Эндпоинт |
| :--- | :--- | :--- |
| **Canvas** | Веб-интерфейс визуализации | [http://localhost:8190](http://localhost:8190) |
| **Orchestrator Health** | Проверка статуса сервиса | [http://localhost:8191/health](http://localhost:8191/health) |
| **Orchestrator State** | Состояния всех роботов (raw) | [http://localhost:8191/state](http://localhost:8191/state) |
| **Orchestrator State (Pretty)** | Состояния роботов (форматированный JSON) | [http://localhost:8191/state?pretty=true](http://localhost:8191/state?pretty=true) |
| **Orchestrator WS** | WebSocket для Canvas | `ws://localhost:8191/ws` |
| **Orchestrator Order** | Отправка приказа роботу | `POST http://localhost:8191/order` |

---

## Управление

| Действие | Как |
| :--- | :--- |
| **Выбрать робота** | Клик по строке в таблице |
| **Управлять вручную** | Кнопка **▶ Управлять** → `WASD` |
| **Выйти из ручного режима** | Кнопка **■ Выход** или `Esc` |
| **Дать приказ** | `curl -X POST http://localhost:8191/order ...` |
| **Переключить карту** | `MAP=stress docker compose up -d` |

> `WASD` работает в любой раскладке клавиатуры (используется `e.code`, а не `e.key`).

---

## Архитектура

```text
┌──────────────────────────────────────────────────────────┐
│  Canvas (браузер, порт 8190)                             │
│  ├─ карта из YAML (узлы, рёбра, стены)                   │
│  ├─ роботы (позиция, статус, миссия)                     │
│  ├─ таблица статусов                                     │
│  └─ WASD + выбор робота                                  │
└──────────────┬───────────────────────────────────────────┘
               │ WebSocket (ws://localhost:8191/ws)
               ▼
┌──────────────────────────────────────────────────────────┐
│  Оркестратор (FastAPI, порт 8191)                        │
│  ├─ YAML: nodes, edges, bounds, robots                   │
│  ├─ Dijkstra: find_path(start, goal)                     │
│  ├─ /order: приказ роботу                                │
│  ├─ /state: состояния всех роботов                       │
│  └─ ROS 2 нода: PUB /robot_<id>/order, /robot_<id>/cmd_vel│
└──────────────┬───────────────────────────────────────────┘
               │ ROS 2 (DDS)
               ▼
┌──────────────────────────────────────────────────────────┐
│  robot_driver (N экземпляров, контейнер ros)             │
│  ├─ SUB /robot_<id>/order (приказ)                       │
│  ├─ SUB /robot_<id>/cmd_vel (ручное управление)          │
│  ├─ SUB /odom_<id> (позиция из Gazebo)                   │
│  ├─ PUB /robot_<id>/state (состояние)                    │
│  └─ PUB /model/turtle_<id>/joint/.../cmd_vel (в Gazebo)  │
└──────────────┬───────────────────────────────────────────┘
               │ ROS 2 (DDS)
               ▼
┌──────────────────────────────────────────────────────────┐
│  ros_gz_bridge (контейнер gazebo)                        │
│  ├─ /odom_<id>: ROS 2 ↔ Gazebo                           │
│  └─ /model/turtle_<id>/joint/.../cmd_vel: ROS 2 ↔ Gazebo │
└──────────────┬───────────────────────────────────────────┘
               │ Gazebo Transport
               ▼
┌──────────────────────────────────────────────────────────┐
│  Gazebo Harmonic (headless)                              │
│  ├─ стены (внешние + внутренние, из YAML)                │
│  ├─ turtle_robot_1 (JointController + OdometryPublisher) │
│  └─ turtle_robot_2 (JointController + OdometryPublisher) │
└──────────────────────────────────────────────────────────┘
```

---

## Конфигурация

### `.env`

```env
ROS_DOMAIN_ID=0
DISPLAY=:0
WEB_PORT=8190
ORCHESTRATOR_PORT=8191
ROSBRIDGE_PORT=8192
MAP=simple
```

### `config_nodes_edges/simple.yaml` / `stress.yaml`

```yaml
name: simple
bounds:
  x_min: -2
  y_min: -2
  x_max: 14
  y_max: 10
  wall_thickness: 0.3
nodes:
  A1: {x: 0, y: 8}
  A2: {x: 2, y: 8}
  ...
parking:
  P1: {x: 12, y: 6}
  ...
edges:
  - [A1, A2]
  - [A2, A3]
  ...
robots:
  robot_1:
    start_node: C1
    start_x: 0
    start_y: 4
    start_yaw: 0
    color: "#4af"
  robot_2:
    start_node: C5
    start_x: 8
    start_y: 4
    start_yaw: 0
    color: "#f4a"
```

> Внутренние стены генерируются автоматически: там, где между соседними узлами нет ребра, ставится стена. Это не даёт роботам срезать по диагонали.

---

## Генерация мира

```bash
# Сгенерировать SDF + JSON стен для simple
python3 scripts/gen_world.py simple

# Для stress
python3 scripts/gen_world.py stress
```

**Выход:**
* `gazebo/warehouse_simple.sdf` — мир для Gazebo (стены + роботы)
* `config_nodes_edges/walls_simple.json` — стены для Canvas

---

## API оркестратора

| Метод | Путь | Описание |
| :--- | :--- | :--- |
| `GET` | `/health` | `{status, map, robots}` |
| `GET` | `/state` | Состояния всех роботов (поддерживает `?pretty=true`) |
| `POST` | `/order` | Приказ: `{robot_id, target, start}` $\rightarrow$ `{ok, path}` |
| `WS` | `/ws` | Canvas: карта + состояния + `cmd_vel` / `resume` |

### Пример: дать приказ

```bash
curl -X POST http://localhost:8191/order \
  -H "Content-Type: application/json" \
  -d '{"robot_id": "robot_1", "target": "C5", "start": "C1"}'
```

**Ответ:**

```json
{
  "ok": true,
  "path": ["C1", "C2", "C3", "C4", "C5"]
}
```

### Пример: проверить состояние

```bash
# Форматированный вывод
curl "http://localhost:8191/state?pretty=true"
```

**Ответ:**

```json
{
  "robots": [
    {
      "id": "robot_1",
      "x": 0.03,
      "y": 4.00,
      "status": "idle",
      "mission": []
    },
    {
      "id": "robot_2",
      "x": 8.03,
      "y": 4.00,
      "status": "idle",
      "mission": []
    }
  ]
}
```

---

## ROS 2 топики

| Топик | Тип | Направление |
| :--- | :--- | :--- |
| `/robot_<id>/order` | `String` | оркестратор $\rightarrow$ нода |
| `/robot_<id>/cmd_vel` | `Twist` | оркестратор $\rightarrow$ нода |
| `/robot_<id>/state` | `String` | нода $\rightarrow$ оркестратор |
| `/odom_<id>` | `Odometry` | Gazebo $\rightarrow$ нода |
| `/model/turtle_<id>/joint/joint_left/cmd_vel` | `Float64` | нода $\rightarrow$ Gazebo |
| `/model/turtle_<id>/joint/joint_right/cmd_vel` | `Float64` | нода $\rightarrow$ Gazebo |

---

## Статусы роботов

| Статус | Значение |
| :--- | :--- |
| `idle` | Стоит, нет миссии |
| `moving` | Едет по миссии |
| `done` | Доехал до цели |
| `manual` | Ручное управление (WASD) |
| `error` | Ошибка (узел не найден, застрял) |

---

## Ограничения

* **Запуск только на localhost.** Canvas подключается к `ws://localhost:8191/ws`. Для доступа с другого хоста нужно заменить `localhost` на IP в `web/index.html` и открыть порт `8191` в файрволе.
* **Headless Gazebo.** GUI не используется (нет GPU/дисплея в контейнере). Визуализация — через Canvas.
* **Один робот в ручном режиме.** При выборе «Управлять» у другого робота предыдущий автоматически выходит из ручного режима.
* **Блокировки узлов/рёбер не реализованы.** Роботы могут ехать по одному маршруту и сталкиваться. В планах — добавление блокировок в оркестратор.
* **«Тёмный цех» не реализован.** В планах — фоновый таск со случайными приказами.

---

## Структура проекта

```text
ROS_test_project/
├── config_nodes_edges/          # YAML-конфиги карт
│   ├── simple.yaml
│   ├── stress.yaml
│   ├── walls_simple.json        # стены для Canvas (генерируется)
│   └── walls_stress.json
├── docker/                      # Dockerfile'ы
│   ├── Dockerfile.orchestrator
│   ├── Dockerfile.ros
│   └── Dockerfile.web
├── gazebo/                      # SDF-миры (генерируются)
│   ├── warehouse_simple.sdf
│   └── warehouse_stress.sdf
├── orchestrator/                # FastAPI + ROS 2 нода
│   ├── main.py
│   └── requirements.txt
├── ros_ws/                      # ROS 2 workspace
│   └── src/robot_sim/
│       └── robot_sim/
│           └── robot_driver.py
├── scripts/
│   └── gen_world.py             # генератор SDF + JSON стен
├── web/                         # Canvas
│   ├── index.html
│   └── favicon.svg
├── docker-compose.yml
├── .env
└── README.md
```

---

## Отладка

### Проверить, что всё живо

```bash
docker compose ps
curl http://localhost:8191/health
```

### Проверить, что ноды публикуют состояние

```bash
docker compose exec ros bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /robot_1/state --once"
```

### Проверить, что команды доходят до Gazebo

```bash
docker compose exec gazebo bash -c "source /opt/ros/jazzy/setup.bash && gz topic -e -t /model/turtle_robot_1/joint/joint_left/cmd_vel"
```

### Логи

```bash
docker compose logs -f orchestrator
docker compose logs -f gazebo
docker compose logs -f ros
```

### Перегенерировать мир

```bash
python3 scripts/gen_world.py simple
MAP=simple docker compose down gazebo
MAP=simple docker compose up -d gazebo
```