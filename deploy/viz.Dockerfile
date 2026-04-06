FROM mycobot-ros2-base:local

ENV PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app/robotic-arm-mcp \
    VIZ_MODE=x11 \
    NOVNC_PORT=6080

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    novnc \
    ros-jazzy-rviz2 \
    tini \
    websockify \
    x11vnc \
    xauth \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY robotic-arm-mcp /app/robotic-arm-mcp
COPY deploy/entrypoints/viz-entrypoint.sh /usr/local/bin/viz-entrypoint.sh

RUN chmod +x /usr/local/bin/viz-entrypoint.sh

WORKDIR /app/robotic-arm-mcp

ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/viz-entrypoint.sh"]

