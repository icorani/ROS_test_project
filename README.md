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

### 2. Запустить контейнеры

```bash
# Поднять все контейнеры (роботы запускаются автоматически)
docker compose up --build
```

> **Примечание:** Первый запуск займёт 5–10 минут (скачивание образов `ros:jazzy`, `osrf/ros:jazzy-desktop-full`, `nginx:alpine`).
>
> Роботы стартуют автоматически из `scripts/bringup.sh` — по одному экземпляру `robot_driver` на каждую активную парковку. Количество активных роботов регулируется переменной `NUM_ROBOTS` в `.env` (пусто = все парковки). Не превышает количества парковочных точек.

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
| **Отправить домой (на парковку)** | `curl -X POST http://localhost:8191/go_home -d '{"robot_id":"robot_1"}'` |
| **Количество активных роботов** | Переменная `NUM_ROBOTS` в `.env` (пусто = все парковки) |
| **Переключить карту** | `MAP=stress docker compose up -d` |

> `WASD` работает в любой раскладке клавиатуры (используется `e.code`, а не `e.key`).
> Неактивные роботы (количество > `NUM_ROBOTS`) отображаются в таблице серым цветом без кнопок управления.

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
│  robot_driver + ros_gz_bridge (контейнер ros)            │
│  ├─ robot_driver: SUB /robot_<id>/order, /robot_<id>/cmd_vel│
│  ├─ robot_driver: SUB /odom_<id>, PUB /robot_<id>/state   │
│  ├─ ros_gz_bridge: /odom_<id> ROS↔Gazebo                 │
│  └─ ros_gz_bridge: /model/turtle_<id>/joint/*/cmd_vel ROS↔Gazebo│
└──────────────┬───────────────────────────────────────────┘
               │ ROS 2 (DDS) / Gazebo Transport
               ▼
┌──────────────────────────────────────────────────────────┐
│  Gazebo Harmonic (headless)                              │
│  ├─ стены (внешние + внутренние, из YAML)                │
│  ├─ turtle_robot_N (JointController + OdometryPublisher) │
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

# Количество активных роботов (пусто = все парковки, не более их числа)
NUM_ROBOTS=
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
```

Секция `robots` не требуется — роботы создаются автоматически из точек `parking` (сортировка по имени: `robot_1` на `P1`, `robot_2` на `P2` и т.д.). Активные роботы — первые `NUM_ROBOTS` парковок, остальные отображаются как `inactive`.

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
| `POST` | `/order` | Приказ: `{robot_id, target, start}` → `{ok, path}` |
| `POST` | `/go_home` | Вернуть робота на свою парковку: `{robot_id}` → `{ok, path, home}` |
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

### Пример: отправить робота домой (на свою парковку)

```bash
curl -X POST http://localhost:8191/go_home \
  -H "Content-Type: application/json" \
  -d '{"robot_id": "robot_1"}'
```

**Ответ:**

```json
{
  "ok": true,
  "path": ["C4", "C3", "C2", "C1", "P1"],
  "home": "P1"
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
| `inactive` | Неактивен (количество роботов > NUM_ROBOTS) |

---

## Ограничения

* **Запуск только на localhost.** Canvas подключается к `ws://localhost:8191/ws`. Для доступа с другого хоста нужно заменить `localhost` на IP в `web/index.html` и открыть порт `8191` в файрволе.
* **Headless Gazebo.** GUI не используется (нет GPU/дисплея в контейнере). Визуализация — через Canvas.
* **Один робот в ручном режиме.** При выборе «Управлять» у другого робота предыдущий автоматически выходит из ручного режима.
* **Количество активных роботов** ограничено переменной `NUM_ROBOTS` и числом парковочных точек (`parking` в YAML карты).

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
│   └── src/
│       ├── robot_sim/
│       │   └── robot_sim/
│       │       └── robot_driver.py
│       └── warehouse_bringup/
│           ├── package.xml
│           ├── setup.py
│           └── launch/
│               └── warehouse_bringup.launch.py
├── scripts/
│   ├── gen_world.py             # генератор SDF + JSON стен
│   └── bringup.sh               # автоматический запуск драйверов и моста
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
## Новые возможности

### Блокировки узлов и ребер
Оркестратор автоматически резервирует узлы и ребра маршрута, чтобы предотвратить столкновения роботов. При блокировке узла/ребра другие роботы ищут альтернативный путь.

### Обнаружение застрявших роботов
Если робот не может двигаться более 2 секунд при активной миссии, оркестратор помечает его как застрявшего и блокирует соответствующую зону графа. Дальнейшие попытки проезда через заблокированную зону запрещены.

### Режим "Тёмный цех"

Режим "Тёмный цех" — стресс-тест orchestration, блокировок и устойчивости к потере связи. При включении фоновый цикл оркестратора (`monitor_tasks` → `do_dark_shop()`) каждые `DARK_SHOP_INTERVAL` (3 с) выбирает одного простаивающего робота и шлёт ему приказ на случайную далёкую цель.

**Как включается.** Кнопка «Тёмный цех» в веб-интерфейсе (`web/index.html`) переключает флаг `dark_shop` через WebSocket (`dark_shop` / `dark_shop_status`). Клиент только переключает флаг; вся генерация приказов реализована на бэкенде.

**Что происходит под капотом.** Оркестратор берёт только активных роботов (`is_active`), без статуса `error`, со статусом `idle`/`done` и временем последней обновления меньше `CONNECTION_TIMEOUT`. Для такого робота выбирается случайная цель из парковочных и всех узлов (кроме текущего), вычисляется путь через `PATHFINDER.find_path(cur, target, LOCKS, rid)` с учётом блокировок, и если путь валиден (`len > 2`), назначается миссия (`status: moving`, непустой `mission`) и отправляется через `NODE.send_order`. Один робот за тик.

**Ограничения.** Приказы шлются только активным и подключённым роботам; застрявшие, неактивные и отключённые роботы не получают приказов. При выключении режима новые приказы перестают приходить.

### Обработка потери связи
При отсутствии обновлений от робота дольше 2.5 секунд оркестратор помечает его как `disconnected`. Приказы и команды для отключившегося робота блокируются.

### Расширенные статусы и ошибки
- `error` - ошибка (застрял, узел не найден, далеко от графа)
- `disconnected` - потеря связи с роботом
- В таблице отображается информация об ошибках

### Улучшенное управление
- Кнопка включения/выключения "Тёмного цеха"
- Визуализация заблокированных узлов (красные) и ребер (пунктирные красные)
- Расширенная таблица с полем ошибок
