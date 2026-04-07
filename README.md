# Robotic Arm

Monorepo para un stack de control de brazos roboticos con agente LLM, servidor MCP, simulacion y despliegue. La implementacion incluida toma como referencia un `myCobot 280 Pi` simulado, pero la arquitectura esta pensada para extenderse a cualquier otro brazo que implemente el mismo contrato MCP.

## Objetivo

Separar responsabilidades para que el sistema sea portable:

- `robotic-arm-agent/`: agente en lenguaje natural, memoria y clientes MCP.
- `robotic-arm-mcp/`: contrato MCP generico para brazos roboticos, safety, perception y backends.
- `deploy/`: contenedores, compose y manifiestos base para correr el stack.
- `docs/`: guias de extension y convenciones del monorepo.

El agente no depende de ROS 2 ni del hardware. El backend del brazo no depende del modelo. El contrato MCP es la frontera estable entre ambos.

## Estructura

```text
.
├── robotic-arm-agent/     # Runtime del agente, memoria y smoke/scenario tests
├── robotic-arm-mcp/       # Servidor MCP, safety, perception y backends del brazo
├── deploy/                # Dockerfiles, compose, entrypoints y manifests k8s
├── docs/                  # Documentacion del monorepo y guia para nuevos brazos
├── PLAN_INTEGRACION.md    # Roadmap funcional y decisiones de arquitectura
├── .env.example           # Variables base para correr el stack containerizado
└── Makefile               # Comandos de build/up/down/smoke
```

## Flujo recomendado

### 1. Desarrollo local del stack

```bash
cp .env.example .env
make build
make up
make smoke
```

La documentacion operativa de contenedores y troubleshooting esta en [deploy/README.md](deploy/README.md).

### 1.1 Primera vez con Docker + AgentScope Studio

Si es tu primera vez y solo queres probar el stack con Docker:

1. Copia el entorno base y ajusta al menos `MCP_API_KEY` y `LLM_API_BASE`.
2. Ejecuta `make build`.
3. Ejecuta `make up`.
4. `make up` levanta `AgentScope Studio`, el MCP, el agente y la visual remota.
5. Al terminar, `make up` imprime las URLs locales mas importantes.
6. Proba primero con `make smoke` o `make scenario`.
7. Cuando quieras una sesion interactiva, hace `attach` al contenedor del agente.

Comandos tipicos:

```bash
cp .env.example .env
make build
make up
make smoke
make scenario PROMPT="Decime que tools tenes disponibles y no muevas nada"
docker compose --env-file .env -f deploy/docker-compose.yml attach arm-agent
```

Notas practicas:

- `make up` ahora levanta `AgentScope Studio` y la visual remota por noVNC.
- Si queres el comportamiento anterior sin UI, usa `make up-core`.
- Si usas un LLM corriendo en el host con Ollama, en `.env` normalmente conviene `LLM_API_BASE=http://host.docker.internal:11434/v1`.

### 2. Desarrollo del agente

```bash
source venv_ai/bin/activate
python robotic-arm-agent/main.py
```

El agente consume tools desde `MCP_URL` y se adapta al brazo llamando `get_capabilities()`.

### 3. Desarrollo del MCP del brazo

```bash
source venv_ai/bin/activate
python robotic-arm-mcp/server.py --backend mock --perception mock
```

El backend `mock` permite validar el contrato sin hardware. La implementacion ROS 2 y la simulacion viven dentro de `robotic-arm-mcp/`.

## Extender a otro brazo

Este repo funciona como base de monorepo. Para soportar otro brazo:

1. Mantene el contrato MCP generico expuesto por `robotic-arm-mcp/server.py`.
2. Implementa un backend nuevo en `robotic-arm-mcp/backends/`.
3. Ajusta `get_capabilities()` para describir correctamente el hardware objetivo.
4. Actualiza `deploy/` solo si el nuevo brazo necesita runtime, drivers o assets distintos.

La guia corta de extension esta en [docs/EXTENDING_TO_ANOTHER_ARM.md](docs/EXTENDING_TO_ANOTHER_ARM.md).

## Estado del repo

- Este workspace ahora esta preparado para vivir como un unico repositorio Git en la raiz.
- `robotic-arm-agent/` deja de ser un repo embebido y pasa a ser un paquete/directorio del monorepo.
- El `origin` a reutilizar es el mismo que ya existia en `robotic-arm-agent`.

## Referencias

- [PLAN_INTEGRACION.md](PLAN_INTEGRACION.md)
- [deploy/README.md](deploy/README.md)
- [robotic-arm-mcp/perception/calibration.example.json](robotic-arm-mcp/perception/calibration.example.json)
