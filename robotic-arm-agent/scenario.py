"""Run scripted prompts against the robot arm agent."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
from typing import Any

from agentscope.message import Msg

# Ensure imports resolve correctly regardless of working directory
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from runtime import create_agent_runtime


def _serialize_message(msg: Msg) -> dict[str, Any]:
    text = msg.get_text_content()
    payload: dict[str, Any] = {
        "name": msg.name,
        "role": msg.role,
        "text": text,
    }
    if text is None:
        payload["content"] = msg.content
    return payload


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scripted Robot Arm agent session")
    parser.add_argument(
        "--prompt",
        action="append",
        required=True,
        help="Prompt a enviar al agente. Repetible para una sesion multi-turno.",
    )
    parser.add_argument(
        "--session-name",
        default="robot-arm-scenario",
        help="Nombre de sesion para AgentScope.",
    )
    return parser.parse_args()


async def _main() -> None:
    args = _parse_args()
    studio_url, reme, _long_term_memory, agent = await create_agent_runtime(
        session_name=args.session_name,
    )

    transcript: list[dict[str, Any]] = []
    try:
        for prompt in args.prompt:
            user_msg = Msg(name="User", role="user", content=prompt)
            reply = await agent(user_msg)
            transcript.append(
                {
                    "prompt": prompt,
                    "reply": _serialize_message(reply),
                }
            )
    finally:
        await reme.close()

    print(
        json.dumps(
            {
                "session_name": args.session_name,
                "studio_url": studio_url or None,
                "transcript": transcript,
            },
            ensure_ascii=True,
        )
    )


if __name__ == "__main__":
    asyncio.run(_main())
