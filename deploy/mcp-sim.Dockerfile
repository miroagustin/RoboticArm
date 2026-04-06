FROM mycobot-ros2-base:local

ENV PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app/robotic-arm-mcp \
    PATH=/opt/mcp-venv/bin:$PATH

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    python3-venv \
    tini \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY robotic-arm-mcp /app/robotic-arm-mcp
COPY deploy/entrypoints/mcp-sim-entrypoint.sh /usr/local/bin/mcp-sim-entrypoint.sh

RUN python3 -m venv --system-site-packages /opt/mcp-venv && \
    /opt/mcp-venv/bin/pip install --no-cache-dir -r /app/robotic-arm-mcp/requirements.txt && \
    chmod +x /usr/local/bin/mcp-sim-entrypoint.sh

WORKDIR /app/robotic-arm-mcp

ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/mcp-sim-entrypoint.sh"]
