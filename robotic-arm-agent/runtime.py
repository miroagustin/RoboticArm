"""Shared runtime helpers for interactive and scripted agent sessions."""

from __future__ import annotations

import os

import agentscope
from agentscope.agent import ReActAgent
from reme.reme_light import ReMeLight

from config import (
    COMPRESSION_TRIGGER_TOKENS,
    SYSTEM_PROMPT,
    create_formatter,
    create_long_term_memory,
    create_model,
    create_toolkit,
)
from reme_memory import ReMeLongTermMemory

_HERE = os.path.dirname(os.path.abspath(__file__))


def init_agentscope_runtime(session_name: str) -> str:
    """Initialize AgentScope and return the active Studio URL, if any."""
    studio_url = os.environ.get("AGENTSCOPE_STUDIO_URL", "http://localhost:3000")
    try:
        agentscope.init(
            project="RobotArmAgent",
            name=session_name,
            studio_url=studio_url,
            logging_path=os.path.join(_HERE, "logs", "agentscope.log"),
        )
        return studio_url
    except Exception:
        agentscope.init(
            project="RobotArmAgent",
            name=session_name,
            logging_path=os.path.join(_HERE, "logs", "agentscope.log"),
        )
        return ""


async def create_agent_runtime(
    session_name: str = "robot-arm-session",
) -> tuple[str, ReMeLight, ReMeLongTermMemory, ReActAgent]:
    """Create the configured agent and its persistent-memory runtime."""
    active_studio_url = init_agentscope_runtime(session_name)
    reme, long_term_memory = create_long_term_memory()
    await reme.start()
    formatter = create_formatter()

    agent = ReActAgent(
        name="RobotOperator",
        sys_prompt=SYSTEM_PROMPT,
        model=create_model(),
        formatter=formatter,
        toolkit=await create_toolkit(),
        long_term_memory=long_term_memory,
        long_term_memory_mode="static_control",
        max_iters=5,
        compression_config=ReActAgent.CompressionConfig(
            enable=True,
            agent_token_counter=formatter.token_counter,
            trigger_threshold=COMPRESSION_TRIGGER_TOKENS,
            keep_recent=6,
        ),
    )
    return active_studio_url, reme, long_term_memory, agent
