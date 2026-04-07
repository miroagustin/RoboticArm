COMPOSE_FILE := deploy/docker-compose.yml
COMPOSE := docker compose --env-file .env -f $(COMPOSE_FILE)
BASE_IMAGE := mycobot-ros2-base:local
K8S_DIR := deploy/k8s

.PHONY: build up up-core urls viz viz-remote smoke scenario down logs config k8s-apply k8s-delete

build:
	docker build -f deploy/base.Dockerfile -t $(BASE_IMAGE) .
	$(COMPOSE) build arm-mcp-sim arm-viz arm-agent

up:
	$(COMPOSE) up -d as-studio arm-mcp-sim arm-agent
	$(COMPOSE) --profile viz-remote up -d arm-viz-remote
	@$(MAKE) urls

up-core:
	$(COMPOSE) up -d arm-mcp-sim arm-agent

urls:
	@set -a; \
	if [ -f .env ]; then . ./.env; fi; \
	set +a; \
	as_studio_port=$${AS_STUDIO_PORT:-3000}; \
	novnc_port=$${NOVNC_PORT:-6080}; \
	llm_api_base=$${LLM_API_BASE:-http://host.docker.internal:8000/v1}; \
	printf '\nURLs locales:\n'; \
	printf '  AgentScope Studio: http://localhost:%s\n' "$$as_studio_port"; \
	printf '  Viz remota:       http://localhost:%s/vnc.html\n' "$$novnc_port"; \
	printf '  MCP healthz:      http://localhost:8010/healthz\n'; \
	printf '  LLM externo:      %s\n\n' "$$llm_api_base"

viz:
	xhost +local:docker
	$(COMPOSE) --profile viz up -d arm-viz

viz-remote:
	$(COMPOSE) --profile viz-remote up -d arm-viz-remote

smoke:
	$(COMPOSE) up -d arm-mcp-sim arm-agent
	$(COMPOSE) exec -T arm-agent python smoke.py check-mcp --probe-tool get_position

scenario:
	$(COMPOSE) up -d arm-mcp-sim arm-agent
	$(COMPOSE) exec -T arm-agent python scenario.py --prompt "$(PROMPT)"

down:
	$(COMPOSE) --profile viz --profile viz-remote down

logs:
	$(COMPOSE) logs -f

config:
	$(COMPOSE) config

k8s-apply:
	kubectl apply -k $(K8S_DIR)

k8s-delete:
	kubectl delete -k $(K8S_DIR)
