"""Agent and model configuration."""

import os
import warnings

from agentscope.formatter import OpenAIChatFormatter
from agentscope.mcp import HttpStatelessClient
from agentscope.model import OpenAIChatModel
from agentscope.token import OpenAITokenCounter
from agentscope.tool import Toolkit
from reme.reme_light import ReMeLight

from reme_memory import ReMeLongTermMemory

warnings.filterwarnings(
    "ignore",
    message="Use `streamable_http_client` instead.",
    category=DeprecationWarning,
)

API_BASE = os.environ.get("LLM_API_BASE", "http://localhost:8000/v1")
MODEL_NAME = os.environ.get("MODEL_NAME", "Qwen/Qwen2.5-7B-Instruct-AWQ")
REME_WORKING_DIR = os.environ.get("REME_WORKING_DIR", ".reme")
MODEL_CONTEXT_WINDOW = int(os.environ.get("MODEL_CONTEXT_WINDOW", "4096"))
MODEL_MAX_OUTPUT_TOKENS = int(os.environ.get("MODEL_MAX_OUTPUT_TOKENS", "512"))
FORMATTER_MAX_TOKENS = int(
    os.environ.get(
        "FORMATTER_MAX_TOKENS",
        str(min(2400, max(1024, MODEL_CONTEXT_WINDOW - 1400))),
    )
)
COMPRESSION_TRIGGER_TOKENS = int(
    os.environ.get(
        "COMPRESSION_TRIGGER_TOKENS",
        str(min(1800, max(768, FORMATTER_MAX_TOKENS - 400))),
    )
)
REME_MAX_RESULTS = int(os.environ.get("REME_MAX_RESULTS", "2"))
REME_MAX_CHARS = int(os.environ.get("REME_MAX_CHARS", "800"))

# MCP server — apunta al robot (o localhost para desarrollo)
MCP_URL = os.environ.get("MCP_URL", "http://localhost:8010/mcp")
MCP_API_KEY = os.environ.get("MCP_API_KEY", "")
REQUIRED_MCP_TOOLS = (
    "get_capabilities",
    "get_position",
    "move_joints",
    "move_cartesian",
    "go_home",
    "emergency_stop",
)

# Comando para iniciar vLLM con soporte de tool calls:
# python3 -m vllm.entrypoints.openai.api_server \
#   --model Qwen/Qwen2.5-7B-Instruct-AWQ \
#   --quantization awq \
#   --gpu-memory-utilization 0.85 \
#   --max-model-len 4096 \
#   --max-num-seqs 16 \
#   --enforce-eager \
#   --enable-auto-tool-choice \
#   --tool-call-parser hermes
#
# Alternativa para Ollama + Gemma 4:
# export LLM_API_BASE=http://localhost:11434/v1
# export MODEL_NAME=gemma4:e2b
# export MODEL_MAX_OUTPUT_TOKENS=768
#
# Nota: con gemma4:e2b en Ollama, los tool calls por /v1/chat/completions
# pueden requerir mas tokens de salida que Qwen porque el modelo suele emitir
# reasoning antes de devolver la llamada a herramienta.

SYSTEM_PROMPT = """\
Sos un operador de brazo robotico. Tu trabajo es interpretar instrucciones \
del usuario en lenguaje natural y usar tus herramientas para controlar el \
brazo robot.

Al iniciar una sesion, llama a get_capabilities() para conocer las \
capacidades del brazo (DOF, limites, gripper, camara, etc.) y adapta tu \
comportamiento en consecuencia.

Herramientas siempre disponibles:
- get_capabilities(): Conocer que puede hacer el brazo.
- get_position(): Ver la posicion actual (joints y cartesiana).
- move_joints(angles, speed): Mover a angulos articulares (grados).
- move_cartesian(x,y,z,rx,ry,rz, speed): Mover a pose cartesiana (mm).
- go_home(speed): Ir a posicion segura.
- emergency_stop(): Parar todo movimiento inmediatamente.

Herramientas opcionales (solo si capabilities lo habilita):
- gripper(action): Si has_gripper=true. "open", "close" o "0"-"100".
- scan_workspace(): Si has_camera=true. Devuelve objetos detectados con \
  posiciones (x,y,z) en mm en el frame base del robot. Usar antes de mover \
  cuando la instruccion referencia objetos del entorno (ej. "agarra la taza").
- plan_and_execute(x,y,z,rx,ry,rz, speed): Si has_planner=true. Planifica \
  evitando obstaculos y luego ejecuta la trayectoria.
- plan_cartesian_path(waypoints, speed): Si has_planner=true. Recorre \
  waypoints cartesianos en linea recta.
- add_collision_object(name, shape, pose): Si has_planner=true. Agrega un \
  obstaculo manual a la escena de colisiones.
- clear_collision_objects(): Si has_planner=true. Limpia la escena de colisiones.

Reglas:
- Antes de mover, llama get_position() para saber donde esta el brazo.
- Si la instruccion menciona objetos reales y has_camera=true, llama \
  scan_workspace() primero para ubicarlos; no adivines coordenadas.
- Si has_planner=true y hay obstaculos potenciales, preferi \
  plan_and_execute() sobre move_cartesian().
- Usa velocidades bajas (10-30) para movimientos de prueba.
- Confirma la operacion al usuario despues de ejecutarla.
- Si la instruccion es ambigua, pedi aclaracion.
- Responde siempre en espanol.
"""


def create_model() -> OpenAIChatModel:
    return OpenAIChatModel(
        model_name=MODEL_NAME,
        api_key="not-needed",
        client_kwargs={"base_url": API_BASE},
        generate_kwargs={"max_tokens": MODEL_MAX_OUTPUT_TOKENS},
    )


def create_formatter() -> OpenAIChatFormatter:
    return OpenAIChatFormatter(
        token_counter=OpenAITokenCounter(MODEL_NAME),
        max_tokens=FORMATTER_MAX_TOKENS,
    )


async def create_toolkit() -> Toolkit:
    """Create the agent toolkit by loading tools from the MCP server."""
    toolkit = Toolkit()
    mcp_client = create_mcp_client()
    await toolkit.register_mcp_client(mcp_client)
    return toolkit


def create_mcp_client() -> HttpStatelessClient:
    """Create the MCP client used by the agent and deploy smoke tests."""
    client_kwargs: dict = {
        "name": "RobotArmMCP",
        "transport": "streamable_http",
        "url": MCP_URL,
    }
    if MCP_API_KEY:
        client_kwargs["headers"] = {"Authorization": f"Bearer {MCP_API_KEY}"}
    return HttpStatelessClient(**client_kwargs)


def create_long_term_memory() -> tuple[ReMeLight, ReMeLongTermMemory]:
    """Create ReMe persistent memory backed by the local LLM."""
    reme = ReMeLight(
        working_dir=REME_WORKING_DIR,
        llm_api_key="not-needed",
        llm_base_url=API_BASE,
        default_as_llm_config={
            "model_name": MODEL_NAME,
            "api_key": "not-needed",
            "client_kwargs": {"base_url": API_BASE},
        },
        default_file_store_config={
            "fts_enabled": True,
            "vector_enabled": False,
        },
        enable_load_env=False,
    )
    return reme, ReMeLongTermMemory(
        reme,
        max_results=REME_MAX_RESULTS,
        max_chars=REME_MAX_CHARS,
    )
