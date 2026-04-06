# Plan de Integración: AgentScope + myCobot 280 Pi

## Objetivo

Conectar el agente ReAct (AgentScope) con el brazo myCobot 280 Pi mediante MCP sobre HTTP. El agente es **agnóstico al hardware** — solo conoce una URL MCP y un contrato genérico de tools. Toda la lógica de ROS 2, pymycobot y seguridad vive en un MCP server separado que corre **en el robot** (o junto al robot).

## Principio de diseño

```
El agente no sabe qué brazo tiene del otro lado.
El robot no sabe qué agente lo controla.
El contrato MCP es la frontera.
```

## Estado actual

```
[AgentScope ReActAgent] → [MCP Server local] → [_send_to_arm() MOCKED]
                               (localhost:8010)           ✗ no llega al brazo

[ROS 2 mycobot_280pi]  → [PyMyCobot SDK] → [Hardware]
                                                    ✓ funciona independiente
```

**Lo que falta:** sacar el MCP server del proyecto del agente y ponerlo del lado del robot, con un contrato genérico de brazo robótico.

## Arquitectura objetivo

```
┌─────────────────────────────────┐         ┌──────────────────────────────────────┐
│         AGENTE (laptop)         │  HTTP   │         ROBOT (Pi / localhost)        │
│                                 │  MCP    │                                      │
│  Usuario ──► ReActAgent         │────────►│  MCP Server (FastMCP)                │
│              (AgentScope)       │         │    ├── move_joints()                  │
│              │                  │◄────────│    ├── move_cartesian()               │
│              ├── ReMe (memoria) │         │    ├── get_position()                 │
│              └── vLLM (LLM)    │         │    ├── gripper()                      │
│                                 │         │    ├── emergency_stop()               │
│  config.py:                     │         │    └── get_capabilities()             │
│    MCP_URL = "http://robot:8010"│         │         │                             │
│                                 │         │    Capa de seguridad (safety.py)      │
│  Sin ROS 2. Sin pymycobot.     │         │         │                             │
│  Sin dependencias de hardware.  │         │    Backend (intercambiable):          │
│                                 │         │      ├── ROS 2 (mycobot_280pi)       │
└─────────────────────────────────┘         │      ├── pymycobot directo           │
                                            │      └── simulación mock             │
                                            │                                      │
                                            │  ROS 2 Jazzy + MoveIt2 (opcional)    │
                                            │         │                            │
                                            │    PyMyCobot SDK → Hardware serial   │
                                            └──────────────────────────────────────┘
```

**Dos proyectos separados:**
- `agente-personal/` — el agente, solo sabe de MCP sobre HTTP
- `mycobot-mcp-server/` — el MCP server del robot, sabe de ROS 2 y pymycobot

---

## Contrato MCP genérico para brazos robóticos

Este contrato es la pieza más importante. Cualquier brazo de N-DOF debería poder implementarlo.

### Tools obligatorias

| Tool | Parámetros | Retorno | Descripción |
|---|---|---|---|
| `get_capabilities()` | — | `{dof, joint_names, joint_limits, workspace_bounds, has_gripper, ...}` | Autodescripción del brazo |
| `get_position()` | — | `{joints: float[], cartesian: {x,y,z,rx,ry,rz}}` | Estado actual completo |
| `move_joints(angles, speed)` | `angles: float[]`, `speed: float (0-100)` | `{success, final_position}` | Mover a ángulos articulares (grados) |
| `move_cartesian(x,y,z,rx,ry,rz, speed)` | coords + `speed: float` | `{success, final_position}` | Mover a pose cartesiana (mm, grados) |
| `go_home(speed)` | `speed: float` | `{success}` | Ir a posición home segura |
| `emergency_stop()` | — | `{success}` | Detener todo movimiento inmediatamente |

### Tools opcionales (según capabilities)

| Tool | Condición | Descripción |
|---|---|---|
| `gripper(action)` | `has_gripper: true` | `"open"` / `"close"` / valor 0-100 |
| `plan_and_execute(x,y,z,rx,ry,rz)` | `has_planner: true` | MoveIt2 trayectoria con evasión de colisiones |
| `scan_workspace()` | `has_camera: true` | Lista de objetos detectados con posición |
| `freedrive(enable)` | `has_freedrive: true` | Activar/desactivar modo de enseñanza manual |
| `record_trajectory(name)` | `has_freedrive: true` | Grabar movimiento manual como skill |
| `replay_trajectory(name, speed)` | — | Reproducir skill grabada |

### Ejemplo de `get_capabilities()` para myCobot 280 Pi

```json
{
  "name": "myCobot 280 Pi",
  "dof": 6,
  "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
  "joint_limits": {
    "j1": [-165, 165], "j2": [-165, 165], "j3": [-165, 165],
    "j4": [-165, 165], "j5": [-165, 165], "j6": [-175, 175]
  },
  "max_payload_g": 250,
  "reach_mm": 280,
  "has_gripper": true,
  "has_planner": false,
  "has_camera": false,
  "has_freedrive": true,
  "backend": "ros2"
}
```

El agente llama `get_capabilities()` al conectarse y adapta su razonamiento. Si el brazo tiene 4 DOF o no tiene gripper, el agente lo sabe y no intenta cosas imposibles.

---

## Fase 1 — MCP Server del robot (simulación)

**Objetivo:** Crear el MCP server como proyecto independiente con backend mock, validar el contrato desde el agente.

### 1.1 Crear proyecto `mycobot-mcp-server`

```
mycobot-mcp-server/
├── server.py              # FastMCP — expone las tools del contrato
├── safety.py              # Validación de límites, rate limiting, e-stop
├── backends/
│   ├── __init__.py
│   ├── base.py            # ArmBackend ABC — interfaz que todos implementan
│   ├── mock.py            # Backend mock (para desarrollo del agente)
│   └── ros2.py            # Backend ROS 2 (para myCobot real/simulado)
├── config.py              # Puerto, backend activo, límites
└── requirements.txt       # fastmcp, pymycobot, etc.
```

**Backend ABC:**
```python
class ArmBackend(ABC):
    @abstractmethod
    def get_capabilities(self) -> dict: ...
    @abstractmethod
    def get_position(self) -> dict: ...
    @abstractmethod
    def move_joints(self, angles: list[float], speed: float) -> dict: ...
    @abstractmethod
    def move_cartesian(self, x, y, z, rx, ry, rz, speed) -> dict: ...
    @abstractmethod
    def go_home(self, speed: float) -> dict: ...
    @abstractmethod
    def emergency_stop(self) -> dict: ...
```

Cada backend implementa esta interfaz. `server.py` delega a `backend` sin saber qué hay detrás.

### 1.2 Capa de seguridad

La seguridad vive en el MCP server, **no en el agente**. El robot se protege a sí mismo.

**Autenticación HTTP por API key:**

El MCP server requiere un header `Authorization: Bearer <API_KEY>` en cada request. Sin key válida, responde 401.

```python
# server.py — middleware de autenticación
API_KEY = os.environ["MCP_API_KEY"]  # nunca hardcodeada

@app.middleware
async def auth_middleware(request, call_next):
    token = request.headers.get("Authorization", "").removeprefix("Bearer ")
    if not hmac.compare_digest(token, API_KEY):
        return Response(status_code=401)
    return await call_next(request)
```

Del lado del agente:
```python
# config.py
MCP_URL = "http://robot:8010/mcp"
MCP_API_KEY = os.environ["MCP_API_KEY"]  # se pasa al HttpStatelessClient
```

Esto es simple, efectivo, y evita que cualquiera en la red pueda mover el brazo. Para producción se puede escalar a mTLS si fuera necesario.

**Validación de movimientos:**

```python
# safety.py — se ejecuta ANTES de pasar al backend

class SafetyLayer:
    def validate_joints(self, angles, limits) -> None:
        """Rechaza ángulos fuera de rango."""
    def validate_speed(self, speed, max_speed) -> None:
        """Capea velocidad al máximo permitido."""
    def validate_cartesian(self, coords, workspace_bounds) -> None:
        """Rechaza posiciones fuera del workspace."""
    def check_rate_limit(self) -> None:
        """Máximo N comandos por segundo."""
```

### 1.3 Adaptar el agente

Cambios mínimos en `agente-personal/`:

| Archivo | Cambio |
|---|---|
| `config.py` | `MCP_URL` apunta al robot (ej: `http://192.168.1.50:8010/mcp`) |
| `config.py` | Actualizar `SYSTEM_PROMPT` para usar las tools genéricas |
| `mcp_server/` | Eliminar — ya no vive acá |

El agente solo necesita `agentscope`, `vllm`, `reme-ai`. Cero dependencias de robótica.

### 1.4 Validar con backend mock

1. Levantar `mycobot-mcp-server` con `--backend mock`
2. Levantar el agente apuntando al MCP server
3. Probar: "qué brazo tenés?", "mové a la posición home", "mové 5cm a la izquierda"
4. Verificar que `get_capabilities()` se llama al inicio y el agente se adapta

### 1.5 Entregable Fase 1

- Proyecto `mycobot-mcp-server/` funcional con backend mock
- Contrato MCP genérico definido y documentado
- `agente-personal/` sin dependencias de hardware
- Agente se conecta por HTTP y controla el "brazo" mock

---

## Fase 2 — Backend ROS 2 (simulación real)

**Objetivo:** Implementar `backends/ros2.py` para conectar con los nodos ROS 2 del myCobot.

### 2.1 Implementar `backends/ros2.py`

```python
class ROS2Backend(ArmBackend):
    def __init__(self, node_name="mcp_arm_client"):
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.cli_set_angles = self.node.create_client(SetAngles, '/set_angles')
        self.cli_set_coords = self.node.create_client(SetCoords, '/set_coords')
        self.cli_get_angles = self.node.create_client(GetAngles, '/get_angles')
        self.cli_get_coords = self.node.create_client(GetCoords, '/get_coords')
        self.cli_gripper = self.node.create_client(GripperStatus, '/set_gripper')
        # spin en thread separado
        self._spin_thread = Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._spin_thread.start()
```

### 2.2 Nodo mock de simulación

Para simular sin hardware serial, crear un nodo que:
- Acepte los mismos servicios que `listen_real_service.py`
- Mantenga estado interno de ángulos/coordenadas
- Publique en `/joint_states` (para RViz)

```
mycobot-mcp-server/
└── sim/
    └── sim_driver_node.py    # Nodo ROS 2 mock que simula el brazo
```

Esto permite validar todo el pipeline sin el robot físico.

### 2.3 Validar con RViz

1. Lanzar `sim_driver_node.py` + `robot_state_publisher` (URDF del myCobot)
2. Lanzar MCP server con `--backend ros2`
3. Lanzar RViz2
4. Desde el agente: "mové joint 1 a 45 grados"
5. Ver el brazo moverse en RViz

### 2.4 Entregable Fase 2

- `backends/ros2.py` funcional
- Nodo de simulación que publica en `/joint_states`
- Brazo se mueve en RViz controlado por lenguaje natural

---

## Fase 3 — Launch orquestado con contenedores (simulación)

**Objetivo:** Empaquetar el stack de simulación (agente + MCP+sim + visualizador) en contenedores orquestables. Un solo comando (`docker compose up`) levanta todo de forma reproducible. El LLM queda **fuera del scope** de orquestación — se asume como servicio externo al que el agente apunta por URL (host, otro compose, o servicio remoto).

**Por qué ahora:** tras Fase 2 el stack corre en 4 procesos que requieren 4 terminales, `source` manual de ROS 2 y coordinación del orden de arranque. Containerizar antes de sumar percepción / MoveIt / hardware evita que la complejidad operacional explote.

**Principio:** el contrato MCP ya aisló al agente del robot. Ahora el empaquetado aísla a cada componente del entorno de desarrollo del host.

### 3.1 Topología de servicios

Tres contenedores, LLM fuera:

```
┌────────────────┐  HTTP/MCP  ┌─────────────────────┐  ROS 2 DDS  ┌──────────────┐
│   arm-agent    │───────────▶│     arm-mcp-sim     │◀───────────▶│   arm-viz    │
│ AgentScope     │            │ MCP server          │             │ RViz2        │
│ ReActAgent     │            │ + sim_driver_node   │             │ + RSP (URDF) │
│ ReMe           │            │ (ROS 2 Jazzy)       │             │ (ROS 2 Jazzy)│
│                │            │                     │             │              │
│ LLM_URL ───────┼─▶ host/ext │  :8010/mcp          │             │   X11 / VNC  │
└────────────────┘            └─────────────────────┘             └──────────────┘
```

| Servicio | Imagen base | Rol | Egress |
|---|---|---|---|
| `arm-agent` | `python:3.12-slim` | Cliente MCP + ReAct + memoria ReMe | `MCP_URL`, `LLM_API_BASE` |
| `arm-mcp-sim` | `mycobot-ros2-base` (custom) | MCP server + sim_driver_node | DDS interno al compose |
| `arm-viz` | `mycobot-ros2-base` + RViz2 | Visualización URDF | DDS + X11/VNC |

### 3.2 Estrategia de imágenes

**Imagen base compartida** `mycobot-ros2-base`: parte de `ros:jazzy-ros-base`, clona `elephantrobotics/mycobot_ros2` + `mycobot_interfaces` y corre `colcon build --symlink-install`. Se construye una sola vez (~10 min) y se reusa en `arm-mcp-sim` y `arm-viz` vía `FROM`. Evita recompilar el workspace dos veces.

```
deploy/
├── base.Dockerfile         # mycobot-ros2-base (workspace precompilado)
├── mcp-sim.Dockerfile      # FROM mycobot-ros2-base + código MCP + sim
├── viz.Dockerfile          # FROM mycobot-ros2-base + rviz2 + launcher
└── agent.Dockerfile        # FROM python:3.12-slim (sin ROS)
```

`arm-agent` sin ROS 2 → ~200 MB. `arm-mcp-sim` y `arm-viz` comparten capas del workspace, así que la segunda imagen pesa apenas el delta.

**Entrypoint de `arm-mcp-sim`:** `tini` como PID 1, hace `source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash`, lanza `sim/sim_driver_node.py` en background y `server.py --backend ros2` en foreground. Un solo contenedor con dos procesos supervisados (el driver sim y el MCP están acoplados, no tiene sentido separarlos).

### 3.3 Red y descubrimiento ROS 2

`arm-mcp-sim` y `arm-viz` hablan DDS entre sí. El discovery multicast puede fallar en bridge networks de Docker. Solución portable:

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` en ambos
- `CYCLONEDDS_URI=file:///config/cyclonedds.xml` con **peers unicast** explícitos (nombres del compose resueltos por DNS: `arm-mcp-sim`, `arm-viz`)
- `ROS_DOMAIN_ID` compartido (ej: `42`)

Fallback de debug: `network_mode: host` en ambos servicios — funciona en Linux, rompe portabilidad y k8s.

El agente habla HTTP a `arm-mcp-sim:8010/mcp`. Nunca toca DDS. Esa es toda la gracia del contrato MCP.

### 3.4 GUI passthrough para RViz

Tres niveles según entorno:

| Entorno | Mecanismo | Config clave |
|---|---|---|
| Dev Linux local | X11 socket | mount `/tmp/.X11-unix` + `DISPLAY` env + `xhost +local:docker` |
| Dev remoto / Mac / k8s | noVNC sidecar | Xvfb + x11vnc + noVNC en la imagen, browser en `:6080` |
| CI headless | Xvfb | screenshot de RViz para smoke tests visuales |

Default: X11 socket (dev Linux local — el entorno del user). Profile `viz-remote` activa la variante noVNC.

### 3.5 `docker-compose.yml` (resumen)

```yaml
services:
  arm-mcp-sim:
    build: { context: ., dockerfile: deploy/mcp-sim.Dockerfile }
    environment:
      - MCP_HOST=0.0.0.0
      - MCP_PORT=8010
      - MCP_BACKEND=ros2
      - MCP_API_KEY=${MCP_API_KEY}
      - MCP_SAFETY_PROFILE=testing
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - CYCLONEDDS_URI=file:///config/cyclonedds.xml
    volumes:
      - ./deploy/cyclonedds.xml:/config/cyclonedds.xml:ro
    ports: ["8010:8010"]
    healthcheck:
      test: ["CMD", "curl", "-fsS", "http://localhost:8010/mcp"]
      interval: 5s
      retries: 10

  arm-viz:
    profiles: ["viz"]
    build: { context: ., dockerfile: deploy/viz.Dockerfile }
    depends_on:
      arm-mcp-sim: { condition: service_healthy }
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - CYCLONEDDS_URI=file:///config/cyclonedds.xml
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./deploy/cyclonedds.xml:/config/cyclonedds.xml:ro

  arm-agent:
    build: { context: ., dockerfile: deploy/agent.Dockerfile }
    depends_on:
      arm-mcp-sim: { condition: service_healthy }
    environment:
      - MCP_URL=http://arm-mcp-sim:8010/mcp
      - MCP_API_KEY=${MCP_API_KEY}
      - LLM_API_BASE=${LLM_API_BASE}     # ej: http://host.docker.internal:8000/v1
    extra_hosts: ["host.docker.internal:host-gateway"]
    volumes:
      - ./robotic-arm-agent/.reme:/app/robotic-arm-agent/.reme   # memoria persistente
    stdin_open: true
    tty: true
```

Decisiones clave:

- `profiles: [viz]` → RViz es opcional (`docker compose --profile viz up`)
- `depends_on` con `service_healthy` → orden de arranque garantizado
- Volume para `.reme` → la memoria del agente sobrevive a `compose down`
- `host.docker.internal` + `extra_hosts` → el agente alcanza el vLLM del host sin publicar puertos extra
- Sin `--gpus` en ninguno de los tres → nadie del stack de simulación requiere GPU (el LLM sí, pero está fuera)

### 3.6 Secrets y configuración

- `.env` (git-ignored) con `MCP_API_KEY`, `LLM_API_BASE`, `DISPLAY`
- `.env.example` committeado con valores dummy
- `deploy/cyclonedds.xml` versionado (no es secret, es config)
- Para k8s: `Secret` para la API key, `ConfigMap` para el perfil DDS y para `cyclonedds.xml`

### 3.7 Variante Kubernetes

Abordar solo cuando el compose esté sólido. Dos `Deployment` principales (`arm-mcp-sim`, `arm-agent`) + `arm-viz` opcional como `Deployment` con `Service NodePort` (acceso por browser vía noVNC).

```
deploy/k8s/
├── namespace.yaml
├── configmap-cyclonedds.yaml
├── secret-mcp-key.yaml          # kubectl create secret generic arm-mcp-key ...
├── mcp-sim.deployment.yaml
├── mcp-sim.service.yaml          # ClusterIP :8010 + headless service para DDS
├── agent.deployment.yaml
├── viz.deployment.yaml           # con noVNC, no X11
└── viz.service.yaml              # NodePort :6080
```

Decisiones clave de k8s:

- DDS cross-pod usa **CycloneDDS unicast** con peers resueltos vía DNS k8s (`arm-mcp-sim.arm.svc.cluster.local`). Multicast no es confiable en CNIs estándar.
- `arm-viz` **siempre** con noVNC en k8s — un cluster no tiene `$DISPLAY`.
- GPU no es requisito (RViz es CPU-only). Si Fase 5 suma Gazebo, agregar `resources.limits: nvidia.com/gpu: 1` + NVIDIA Device Plugin.
- Hardware real (Fase 6) **no** corre en el cluster — el MCP se despliega en el Pi. El cluster solo hospeda simulación. Esto se documenta explícitamente para evitar tentaciones.

### 3.8 Makefile

```makefile
.PHONY: build up viz down logs k8s-apply k8s-delete

build:       ; docker compose -f deploy/docker-compose.yml build
up:          ; docker compose -f deploy/docker-compose.yml up -d arm-mcp-sim arm-agent
viz:         ; xhost +local:docker && docker compose -f deploy/docker-compose.yml --profile viz up -d
down:        ; docker compose -f deploy/docker-compose.yml --profile viz down
logs:        ; docker compose -f deploy/docker-compose.yml logs -f
k8s-apply:   ; kubectl apply -k deploy/k8s
k8s-delete:  ; kubectl delete -k deploy/k8s
```

### 3.9 Validación

1. `make build` — tres imágenes construidas (la primera vez ~10 min por `colcon build`, luego cache)
2. `make up` — `arm-mcp-sim` pasa healthcheck, `arm-agent` conectado (`docker compose logs arm-agent` muestra que cargó las tools por MCP)
3. `docker attach arm-agent` → "andá a home" → comando procesado, log del `sim_driver_node` visible en `arm-mcp-sim`
4. `make viz` — RViz se abre y muestra el brazo en home
5. "mové joint 1 a 45 grados" → movimiento visible en RViz
6. `make down && make up` → la memoria ReMe persiste (el agente recuerda la instrucción anterior)
7. `make k8s-apply` en minikube/k3s → noVNC accesible por browser, mismos comandos funcionan

### 3.10 Fuera de scope de esta fase

- **vLLM containerizado**: requiere GPU passthrough + ~6 GB VRAM. Se mantiene en host para flexibilidad (el user comparte GPU con otras cosas).
- **Gazebo containerizado**: GPU + display pesado; se aborda si Fase 5 lo requiere.
- **Observability** (Prometheus / Loki / Grafana): útil pero posterior.
- **CI**: pipeline de build + smoke test del compose es nice-to-have, no bloqueante.

### 3.11 Entregable Fase 3

- `deploy/` con tres Dockerfiles + imagen base compartida, `docker-compose.yml`, `cyclonedds.xml`, `.env.example`
- Manifiestos k8s en `deploy/k8s/` (variante con noVNC para viz)
- `Makefile` en la raíz con targets `build`, `up`, `viz`, `down`, `k8s-apply`
- `deploy/README.md` con setup + troubleshooting (DDS, X11, `host.docker.internal`)
- Un solo comando (`make up && make viz`) levanta todo el stack de simulación
- LLM sigue en host — cambiar `LLM_API_BASE` en `.env` y listo

---

## Fase 4 — Percepción del workspace

**Objetivo:** El agente necesita "ver" qué hay en el workspace.

### 4.1 Percepción como servicio del MCP server

La percepción vive del lado del robot (tiene la cámara), no del agente.

**Opciones (de más simple a más complejo):**

| Opción | Pros | Contras |
|---|---|---|
| **Aruco markers** | Pose 3D precisa, fácil, sin GPU | Requiere pegar markers |
| **YOLOv8-nano** | Detecta objetos genéricos | Solo bounding box 2D, necesita estimar Z |
| **LLM multimodal** | Flexible, describe la escena | Lento (~2s), requiere modelo con visión |

**Recomendación:** Empezar con Aruco (preciso y determinista), agregar YOLO después para objetos sin marker.

### 4.2 Tool `scan_workspace()`

Retorna:
```json
{
  "objects": [
    {"name": "taza", "position": {"x": 150, "y": 80, "z": 0}, "confidence": 0.98},
    {"name": "marker_5", "position": {"x": 200, "y": -50, "z": 10}, "confidence": 1.0}
  ],
  "timestamp": "2026-04-04T15:30:00Z"
}
```

### 4.3 Entregable Fase 4

- `scan_workspace()` en el MCP server del robot
- Detección funcional (al menos Aruco)
- El agente puede razonar sobre objetos reales

---

## Fase 5 — Planificación con MoveIt2

**Objetivo:** Trayectorias con evasión de colisiones.

### 5.1 MoveIt2 como backend avanzado

MoveIt2 vive del lado del robot. El MCP server expone tools de planificación solo si `has_planner: true` en capabilities.

| Tool | Descripción |
|---|---|
| `plan_and_execute(x,y,z,rx,ry,rz)` | Planificar con evasión y ejecutar |
| `plan_cartesian_path(waypoints)` | Trayectoria cartesiana (línea recta) |
| `add_collision_object(name, shape, pose)` | Agregar obstáculo a la escena |
| `clear_collision_objects()` | Limpiar escena |

### 5.2 Integración percepción + planificación

1. `scan_workspace()` detecta objetos
2. Los objetos se agregan automáticamente como collision objects
3. `plan_and_execute()` planifica esquivándolos

### 5.3 Entregable Fase 5

- MoveIt2 integrado como opción en el MCP server
- Escena de colisiones dinámica desde percepción
- Pick & place seguro con evasión

---

## Fase 6 — Hardware real

**Objetivo:** Pasar de simulación a myCobot 280 Pi físico.

### 6.1 Qué cambia

| Componente | Cambio |
|---|---|
| Agente | **Ninguno** — misma URL MCP, mismas tools |
| MCP server | **Ninguno** — mismo código |
| Backend | Cambiar de `mock`/`ros2` simulado a `ros2` con driver real |
| ROS 2 | Lanzar `listen_real_service` en vez del sim_driver |

```bash
# Simulación
python server.py --backend mock

# ROS 2 simulado
python server.py --backend ros2    # con sim_driver_node corriendo

# Hardware real
python server.py --backend ros2    # con listen_real_service corriendo
```

El agente no se entera del cambio. **Esa es la ventaja de esta arquitectura.**

### 6.2 Conexión serial

```bash
# En el robot / Pi
sudo usermod -a -G dialout $USER
ros2 run mycobot_280pi listen_real_service --ros-args -p port:=/dev/ttyAMA0 -p baud:=1000000
```

### 6.3 Seguridad adicional para hardware

La capa de seguridad (`safety.py`) ya está activa desde Fase 1, pero para hardware real:

1. **Speed cap:** 25% máximo durante pruebas iniciales
2. **Soft limits:** 10% menos que los límites físicos reales
3. **E-stop físico:** Verificar que el e-stop del myCobot funciona independientemente del software
4. **Watchdog:** Si no llega un comando en N segundos, parar el brazo
5. **Modo gradual:** Primero probar `get_position()` sin mover nada, luego movimientos pequeños

```python
# Niveles de seguridad configurables
SAFETY_PROFILES = {
    "testing":    {"max_speed": 15, "limit_margin_deg": 15, "watchdog_s": 5},
    "normal":     {"max_speed": 50, "limit_margin_deg": 10, "watchdog_s": 10},
    "production": {"max_speed": 80, "limit_margin_deg": 5,  "watchdog_s": 30},
}
```

### 6.4 Calibración

1. `go_home()` → verificar visualmente que la posición coincide
2. `get_position()` → comparar con medición manual
3. Si hay cámara: calibrar offset eye-in-hand

### 6.5 Entregable Fase 6

- myCobot 280 Pi real controlado por lenguaje natural
- Perfiles de seguridad configurables
- E-stop funcional

---

## Fase 7 — Multi-agente y skills avanzadas

**Objetivo:** Expandir capacidades del agente sin tocar el MCP server del robot.

### 7.1 Sistema multi-agente (lado agente)

```
                    ┌─────────────────┐
                    │  Orchestrator   │  ← Recibe instrucción del usuario
                    │  (ReActAgent)   │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
      ┌──────────┐   ┌──────────┐   ┌──────────┐
      │ Planner  │   │ Perceiver│   │ Executor │
      │ Agent    │   │ Agent    │   │ Agent    │
      └──────────┘   └──────────┘   └──────────┘
      Descompone      Llama           Llama
      tareas en       scan_workspace  move/gripper
      pasos           vía MCP         vía MCP
```

Todos los agentes usan el **mismo MCP server**. La inteligencia está en cómo se coordinan, no en el robot.

### 7.2 Skills aprendidas

1. Usuario activa `freedrive(true)` y mueve el brazo manualmente
2. `record_trajectory("apilar_cajas")` graba la secuencia
3. Más adelante: "hacé lo que te enseñé con las cajas"
4. Agente llama `replay_trajectory("apilar_cajas", speed=50)`

Las trajectories se almacenan en el MCP server del robot (son específicas del hardware).

### 7.3 Portabilidad a otro brazo

Para soportar un brazo nuevo (ej: UR5, Dobot, brazo casero):

1. Crear nuevo backend en `backends/ur5.py` implementando `ArmBackend`
2. Lanzar el MCP server con `--backend ur5`
3. El agente se conecta sin cambios — llama `get_capabilities()` y se adapta

```bash
# myCobot 280 Pi
python server.py --backend ros2 --port 8010

# UR5 (otro día, otro brazo)
python server.py --backend ur5 --port 8010

# Brazo casero con Arduino
python server.py --backend serial_custom --port 8010
```

El agente es siempre el mismo.

### 7.4 Entregable Fase 7

- Pipeline pick & place completo por lenguaje natural
- Sistema multi-agente funcional
- Al menos una skill aprendida por demostración
- Documentación de cómo agregar un backend nuevo

---

## Resumen de fases

```
Fase 1: MCP Server + backend mock          ← COMPLETADO
   │    (contrato genérico, safety layer)
   │
   └── Fase 2: Backend ROS 2 (simulación)
          │
          ├── Fase 3: Launch orquestado (Docker / k8s)
          │      │   agente + mcp+sim + viz, LLM fuera
          │      │
          │      ├── Fase 4: Percepción (cámara)
          │      │
          │      └── Fase 5: MoveIt2 (planificación)
          │
          └── Fase 6: Hardware real
                 │    (solo cambiar qué nodo ROS 2 corre)
                 │
                 └── Fase 7: Multi-agente + skills
```

## Dos proyectos, dos repos

### `agente-personal/` (este repo)

```
agente-personal/
├── main.py               # Entry point
├── config.py             # MCP_URL apunta al robot
├── reme_memory.py        # Memoria persistente
└── (sin mcp_server/)     # Ya no vive acá
```

**Dependencias:** `agentscope`, `vllm`, `reme-ai`
**No depende de:** ROS 2, pymycobot, rclpy

### `mycobot-mcp-server/` (nuevo repo)

```
mycobot-mcp-server/
├── server.py             # FastMCP — tools del contrato genérico
├── safety.py             # Validación, límites, rate limiting
├── config.py             # Backend activo, puerto, perfil de seguridad
├── backends/
│   ├── base.py           # ArmBackend ABC
│   ├── mock.py           # Para desarrollo sin hardware
│   └── ros2.py           # Para myCobot real/simulado vía ROS 2
├── sim/
│   └── sim_driver_node.py  # Nodo ROS 2 mock
└── requirements.txt
```

**Dependencias:** `fastmcp`, `rclpy`, `pymycobot`, `mycobot_interfaces`
**No depende de:** agentscope, vllm, reme-ai

## Stack tecnológico

| Capa | Tecnología | Dónde corre |
|---|---|---|
| Interfaz usuario | Terminal / AgentScope Studio | Laptop |
| LLM | Qwen2.5-7B-AWQ vía vLLM | Laptop (RTX 5070) |
| Framework de agentes | AgentScope v1.0.18 | Laptop |
| Memoria | ReMe (full-text + vectores) | Laptop |
| Protocolo | MCP sobre HTTP | Red local |
| MCP Server | FastMCP | Robot / Pi |
| Seguridad | safety.py | Robot / Pi |
| Middleware robótico | ROS 2 Jazzy | Robot / Pi |
| Planificación | MoveIt2 | Robot / Pi |
| Percepción | YOLOv8-nano / Aruco | Robot / Pi |
| Hardware SDK | pymycobot (serial) | Robot / Pi |
| Hardware | myCobot 280 Pi (6DOF) | Robot |
