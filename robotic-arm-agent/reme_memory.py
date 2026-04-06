"""Adapter that connects ReMeLight with AgentScope's LongTermMemoryBase.

ReMeLight provides persistent file-based memory with hybrid search.
This adapter bridges it into AgentScope's agent memory interface so that
ReActAgent can automatically retrieve past context before reasoning and
record new information after each interaction.
"""

from __future__ import annotations

from typing import Any

from agentscope.memory._long_term_memory._long_term_memory_base import (
    LongTermMemoryBase,
)
from agentscope.message import Msg
from reme.reme_light import ReMeLight


class ReMeLongTermMemory(LongTermMemoryBase):
    """Wraps ReMeLight as an AgentScope LongTermMemoryBase.

    The ReActAgent calls:
    - await retrieve(msg)  — before reasoning, to inject past context
    - await record(msgs)   — after each reply, to persist the conversation
    """

    def __init__(
        self,
        reme: ReMeLight,
        max_results: int = 2,
        max_chars: int = 800,
    ) -> None:
        super().__init__()
        self._reme = reme
        self._max_results = max_results
        self._max_chars = max_chars

    async def retrieve(
        self,
        msg: Msg | list[Msg] | None,
        limit: int = 5,
        **kwargs: Any,
    ) -> str:
        """Search long-term memory for context relevant to the current message."""
        if msg is None:
            return ""

        if isinstance(msg, list):
            query = " ".join(
                m.get_text_content() for m in msg if m is not None
            )
        else:
            query = msg.get_text_content()

        if not query.strip():
            return ""

        response = await self._reme.memory_search(
            query,
            max_results=min(limit, self._max_results),
        )

        # ToolResponse.content is a list of TextBlocks
        texts = [
            block.get("text", "")
            for block in response.content
            if block.get("type") == "text"
        ]
        result = "\n".join(texts).strip()
        if len(result) > self._max_chars:
            result = result[: self._max_chars].rstrip() + "\n...[memoria truncada]"
        return result

    async def record(
        self,
        msgs: list[Msg | None],
        **kwargs: Any,
    ) -> None:
        """Persist the current conversation to long-term memory."""
        valid_msgs = [msg for msg in msgs if msg is not None]
        if not valid_msgs:
            return
        await self._reme.summary_memory(messages=valid_msgs)
