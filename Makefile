COMPOSE_FILE := deploy/docker-compose.yml
COMPOSE := docker compose --env-file .env -f $(COMPOSE_FILE)
BASE_IMAGE := mycobot-ros2-base:local
K8S_DIR := deploy/k8s

.PHONY: build up viz viz-remote smoke scenario down logs config k8s-apply k8s-delete

build:
	docker build -f deploy/base.Dockerfile -t $(BASE_IMAGE) .
	$(COMPOSE) build arm-mcp-sim arm-viz arm-agent

up:
	$(COMPOSE) up -d arm-mcp-sim arm-agent

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
