# Deploy Stage 3

Esta carpeta empaqueta la simulacion del stack en contenedores:

- `arm-agent`: AgentScope + ReMe, sin ROS 2
- `arm-mcp-sim`: MCP server + `sim_driver_node.py`
- `arm-viz`: RViz2 con X11 local
- `arm-viz-remote`: la misma imagen de visualizacion, pero expuesta por noVNC

El LLM queda fuera del `compose`. El agente se conecta por `LLM_API_BASE`.

## Archivos

- `base.Dockerfile`: imagen compartida `mycobot-ros2-base:local`
- `mcp-sim.Dockerfile`: servidor MCP y driver simulado
- `viz.Dockerfile`: RViz2 con soporte X11/noVNC
- `agent.Dockerfile`: agente liviano sin dependencias de ROS
- `cyclonedds.xml`: discovery ROS 2 por unicast entre servicios

## Setup

1. Crear el archivo de entorno:

```bash
cp .env.example .env
```

2. Ajustar como minimo:

```bash
MCP_API_KEY=tu-clave
LLM_API_BASE=http://host.docker.internal:8000/v1
AGENTSCOPE_STUDIO_URL=http://as-studio:3000
AS_STUDIO_PORT=3000
DISPLAY=:0
MCP_PLANNER=moveit2
```

3. Construir imagenes:

```bash
make build
```

La primera vez tarda bastante porque `base.Dockerfile` clona `mycobot_ros2` y corre `colcon build`.

## Uso

### Primer uso con AgentScope Studio

`make up` ahora levanta la UI completa del stack para entorno local:

1. Configurar `.env`
2. Correr `make build`
3. Correr `make up`
4. Abrir las URLs locales impresas por el comando
5. Probar primero `make smoke` o `make scenario`
6. Si queres conversar en vivo con el agente, hacer `attach`

Flujo tipico:

```bash
cp .env.example .env
make build
make up
make smoke
make scenario PROMPT="Decime que tools tenes disponibles y no muevas nada"
```

Sesion interactiva opcional:

```bash
docker compose --env-file .env -f deploy/docker-compose.yml attach arm-agent
```

`make up` imprime, como minimo:

- `AgentScope Studio`: `http://localhost:${AS_STUDIO_PORT}`
- `Viz remota`: `http://localhost:${NOVNC_PORT}/vnc.html`
- `MCP healthz`: `http://localhost:8010/healthz`

Si usas Ollama en el host, un valor comun para `.env` es:

```bash
LLM_API_BASE=http://host.docker.internal:11434/v1
```

Si queres levantar solo MCP + agente, sin las UIs, usa:

```bash
make up-core
```

Levantar MCP + agente:

```bash
make up
```

Validar que el agente ve el MCP y que el contrato minimo esta cargado:

```bash
make smoke
```

Ejecutar un prompt real contra el agente containerizado:

```bash
make scenario PROMPT="Decime que tools tenes disponibles y no muevas nada"
```

Probar herramientas de planificacion de la etapa 5:

```bash
docker compose --env-file .env -f deploy/docker-compose.yml exec -T arm-agent \
  python smoke.py check-mcp \
    --expect-tool plan_and_execute \
    --expect-tool add_collision_object
```

Levantar el planner real de MoveIt2 sin tocar `.env`:

```bash
MCP_PLANNER=moveit2 make up
```

Smoke real de `plan_and_execute` con MoveIt2:

```bash
docker compose --env-file .env -f deploy/docker-compose.yml exec -T arm-agent \
  python smoke.py check-mcp \
    --expect-tool plan_and_execute \
    --probe-tool plan_and_execute \
    --probe-kwargs '{"x": 180, "y": 20, "z": 200, "rx": 180, "ry": 0, "rz": 0, "speed": 10, "sync_with_perception": false}'
```

Validar bloqueo por colision sincronizando percepcion:

```bash
docker compose --env-file .env -f deploy/docker-compose.yml exec -T arm-agent \
  python smoke.py check-mcp \
    --expect-tool plan_and_execute \
    --probe-tool plan_and_execute \
    --probe-kwargs '{"x": 180, "y": 60, "z": 40, "rx": 180, "ry": 0, "rz": 0, "speed": 10, "sync_with_perception": true}'
```

Abrir RViz con X11 local:

```bash
make viz
```

Abrir visualizacion remota por navegador:

```bash
make viz-remote
```

Luego abrir `http://localhost:6080/vnc.html`.

Para interactuar con el agente:

```bash
docker compose --env-file .env -f deploy/docker-compose.yml attach arm-agent
```

Ver logs:

```bash
make logs
```

Ver estado de salud de los servicios:

```bash
docker compose --env-file .env -f deploy/docker-compose.yml ps
```

`arm-agent` queda `healthy` cuando puede listar las tools requeridas del MCP. Ese check no valida el LLM externo: solo readiness del contrato MCP dentro del stack.

Para validar una sesion real con LLM externo y memoria persistente, una secuencia util es:

```bash
make scenario PROMPT="Recorda que el codigo de prueba es MEM-123 y no muevas nada"
make down
make up
make scenario PROMPT="Sin mover el brazo, decime cual era el codigo de prueba"
```

La Fase 5 ahora soporta dos modos de planner con el mismo contrato MCP:

- `MCP_PLANNER=basic`: fallback liviano que sincroniza obstaculos desde `scan_workspace()` y genera una trayectoria simple de "subir-desplazar-bajar".
- `MCP_PLANNER=moveit2`: backend real con MoveIt2/OMPL para `plan_and_execute()` y `plan_cartesian_path()`, manteniendo las tools `add_collision_object()` y `clear_collision_objects()`.

Notas practicas:

- En simulacion, la pose cartesiana devuelta por `get_position()` sigue siendo aproximada porque `sim_driver_node.py` usa FK/IK simplificada para visualizacion. Para validar la trayectoria real del planner, toma como referencia RViz, los joints y la respuesta de `trajectory`.
- Si el robot quedo en un estado articular fuera de los limites de MoveIt2, el backend responde con sugerencia de `go_home()` antes de reintentar.
- En `moveit2`, `plan_cartesian_path()` resuelve una secuencia de segmentos via MoveIt2 manteniendo el contrato MCP; todavia no usa interpolacion cartesiana nativa.

Bajar todo:

```bash
make down
```

## Troubleshooting

### DDS / ROS 2 no descubre peers

- Confirmar `ROS_DOMAIN_ID` identico en `arm-mcp-sim` y `arm-viz`
- Confirmar `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Revisar que `deploy/cyclonedds.xml` este montado en ambos servicios
- Inspeccionar logs de `arm-mcp-sim` y `arm-viz`

Si hace falta aislar el problema, una prueba rapida es correr solo `arm-mcp-sim` y verificar:

```bash
curl http://localhost:8010/healthz
```

Tambien podes verificar desde el contenedor del agente:

```bash
docker compose --env-file .env -f deploy/docker-compose.yml exec -T arm-agent \
  python smoke.py check-mcp --probe-tool get_position
```

### X11 no abre RViz

- Ejecutar `xhost +local:docker`
- Confirmar que `DISPLAY` en `.env` coincide con el host
- Verificar que `/tmp/.X11-unix` exista en el host

Si estas en remoto, Mac o un entorno sin X11 local, usar `make viz-remote`.

### `host.docker.internal` no resuelve

El `compose` ya agrega:

```yaml
extra_hosts:
  - "host.docker.internal:host-gateway"
```

Eso requiere Docker moderno sobre Linux. Si igual falla, reemplaza `LLM_API_BASE` por la IP real del host.

### AgentScope Studio no conecta

No bloquea el arranque del agente. Si Studio no esta levantado, el agente sigue funcionando y solo pierde el dashboard.

### MoveIt2 no encuentra plan valido

- Probar `go_home()` y volver a ejecutar el plan
- Confirmar que `MCP_PLANNER=moveit2`
- Revisar `docker logs deploy-arm-mcp-sim-1` para errores de inicializacion de MoveIt2
- Si usas `sync_with_perception=true`, validar primero `scan_workspace()` para entender que obstaculos entraron a la escena

## Kubernetes

Los manifiestos iniciales viven en `deploy/k8s/` y se aplican con:

```bash
make k8s-apply
```

La variante k8s usa noVNC para visualizacion. Si las imagenes son locales, antes hay que cargarlas al cluster (`minikube image load`, `k3d image import`, etc.).

`arm-agent` usa una `readinessProbe`/`livenessProbe` por `exec` con el mismo smoke check MCP del compose.

`as-studio` ahora vive dentro del cluster y el agente lo consume por DNS interno:

```text
http://as-studio:3000
```

Para abrir AgentScope Studio desde tu maquina local:

```bash
kubectl port-forward -n arm svc/as-studio 3000:3000
```

Y luego abrir:

```text
http://localhost:3000
```
