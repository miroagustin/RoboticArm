"""Smoke checks for the containerized robot arm agent deployment."""

import argparse
import asyncio
import json
from typing import Any
import warnings

from config import MCP_URL, REQUIRED_MCP_TOOLS, create_mcp_client

warnings.filterwarnings(
    "ignore",
    message="Use `streamable_http_client` instead.",
    category=DeprecationWarning,
)


def _serialize_tool_result(result: Any) -> Any:
    """Convert AgentScope MCP results into JSON-serializable content."""
    content = getattr(result, "content", None)
    if content is None:
        return str(result)

    serialized: list[Any] = []
    for item in content:
        if hasattr(item, "text"):
            serialized.append(item.text)
        elif hasattr(item, "model_dump"):
            serialized.append(item.model_dump(mode="json"))
        else:
            serialized.append(str(item))
    return serialized


async def _check_mcp(probe_tool: str | None) -> None:
    await _check_mcp_with_args(probe_tool=probe_tool, probe_kwargs=None, expected_tools=[])


async def _check_mcp_with_args(
    probe_tool: str | None,
    probe_kwargs: dict[str, Any] | None,
    expected_tools: list[str],
) -> None:
    client = create_mcp_client()
    tools = await client.list_tools()
    tool_names = sorted(tool.name for tool in tools)
    missing = sorted(set(REQUIRED_MCP_TOOLS) - set(tool_names))
    if missing:
        raise RuntimeError(
            f"MCP en {MCP_URL} no expone las tools requeridas: {', '.join(missing)}"
        )
    expected_missing = sorted(set(expected_tools) - set(tool_names))
    if expected_missing:
        raise RuntimeError(
            "MCP en "
            f"{MCP_URL} no expone las tools esperadas: {', '.join(expected_missing)}"
        )

    payload: dict[str, Any] = {
        "status": "ok",
        "mcp_url": MCP_URL,
        "tools": tool_names,
    }

    if probe_tool:
        tool_fn = await client.get_callable_function(
            probe_tool,
            wrap_tool_result=False,
        )
        payload["probe_tool"] = probe_tool
        payload["probe_kwargs"] = probe_kwargs or {}
        payload["probe_result"] = _serialize_tool_result(
            await tool_fn(**(probe_kwargs or {}))
        )

    print(json.dumps(payload, ensure_ascii=True))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robot arm agent smoke checks")
    subparsers = parser.add_subparsers(dest="command", required=True)

    check_mcp = subparsers.add_parser(
        "check-mcp",
        help="Verifica conectividad MCP y presencia del contrato requerido",
    )
    check_mcp.add_argument(
        "--probe-tool",
        default=None,
        help="Tool opcional a invocar despues de validar el contrato",
    )
    check_mcp.add_argument(
        "--probe-kwargs",
        default="{}",
        help="JSON object con kwargs para la tool del probe",
    )
    check_mcp.add_argument(
        "--expect-tool",
        action="append",
        default=[],
        help="Tool adicional que debe existir en el MCP. Repetible.",
    )
    return parser.parse_args()


async def _main() -> None:
    args = _parse_args()
    if args.command == "check-mcp":
        try:
            probe_kwargs = json.loads(args.probe_kwargs)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"--probe-kwargs must be valid JSON: {exc}") from exc
        if not isinstance(probe_kwargs, dict):
            raise RuntimeError("--probe-kwargs must decode to a JSON object")
        await _check_mcp_with_args(
            probe_tool=args.probe_tool,
            probe_kwargs=probe_kwargs,
            expected_tools=args.expect_tool,
        )


if __name__ == "__main__":
    asyncio.run(_main())
