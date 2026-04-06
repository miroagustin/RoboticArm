FROM python:3.12-slim

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    REME_WORKING_DIR=/app/robotic-arm-agent/.reme

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    tini \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app/robotic-arm-agent

COPY deploy/agent.requirements.txt /tmp/agent.requirements.txt
RUN pip install -r /tmp/agent.requirements.txt

COPY robotic-arm-agent /app/robotic-arm-agent

RUN mkdir -p /app/robotic-arm-agent/.reme /app/robotic-arm-agent/logs

ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["python", "main.py"]

