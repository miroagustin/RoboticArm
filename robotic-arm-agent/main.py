"""Robot arm agent — controls a robot arm via natural language."""

import asyncio
import os
import sys

# Ensure imports resolve correctly regardless of working directory
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from agentscope.agent import UserAgent

from runtime import create_agent_runtime


async def main() -> None:
    studio_url, reme, _long_term_memory, agent = await create_agent_runtime()

    try:
        if studio_url:
            print(f"AgentScope Studio conectado en {studio_url}")
        else:
            print("AgentScope Studio no disponible, corriendo sin dashboard.")
        user = UserAgent(name="User")

        print("=== Robot Arm Agent ===")
        print("Escribi instrucciones para el brazo robot.")
        print("Ejemplo: 'mové la lapicera 5cm a la izquierda'")
        print("Escribi 'salir' para terminar.\n")

        msg = None
        while True:
            msg = await user(msg)
            if msg.get_text_content().strip().lower() in ("salir", "exit", "quit"):
                print("Chau!")
                break
            msg = await agent(msg)
    finally:
        try:
            await reme.close()
        except Exception:
            pass


if __name__ == "__main__":
    asyncio.run(main())
