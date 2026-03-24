SHELL := /bin/bash

.DEFAULT_GOAL := all

ROOT_DIR := $(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
TRACING_LOG_DIRECTORY := $(ROS_HOME)/tracing
DOCKER_BUILDKIT       ?= 1
DOCKER_CONFIG         ?=

PROJECT  := lttng-scope
ARCH     ?= $(shell uname -m)
TAG      := $(ARCH)_$(shell git rev-parse --short HEAD 2>/dev/null || echo NOHASH)
REPO     ?= $(shell git config --get remote.origin.url 2>/dev/null \
              | sed -e 's|.*github.com[:/]||' -e 's|\.git$$||' \
              | tr '[:upper:]' '[:lower:]')
REPO_LC  := $(shell echo "$(REPO)" | tr '[:upper:]' '[:lower:]')

IMAGE          := $(PROJECT):$(TAG)
REGISTRY_IMAGE := ghcr.io/$(REPO_LC)/$(PROJECT):$(TAG)

USER := $(shell whoami)
UID  := $(shell id -u)
GID  := $(shell id -g)

.PHONY: help
help:
	@printf "Usage: make \033[36m<target>\033[0m\n%s\n" "$$(awk 'BEGIN {FS = \":.*##\"} /^[a-zA-Z0-9_-]+:.*?##/ { gsub(/\\$$/, "", $$2); printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) }' $(MAKEFILE_LIST) | sort | uniq | sed 's/\$$//' | sed 's/\\$$//' )"

.PHONY: build
build: ## Pull arch-tagged image if available, build only if not found
	docker pull "$(REGISTRY_IMAGE)" 2>/dev/null && \
	    docker tag "$(REGISTRY_IMAGE)" "$(IMAGE)" || \
	docker build \
	    --platform linux/$(ARCH) \
	    --cache-from "$(REGISTRY_IMAGE)" \
	    -t "$(IMAGE)" \
	    -f Dockerfile \
	    .

.PHONY: push
push: ## Push LTTng Scope image to ghcr.io
	docker tag  "$(IMAGE)" "$(REGISTRY_IMAGE)"
	docker push "$(REGISTRY_IMAGE)"

.PHONY: clean
clean: ## Clean all build artifacts including docker image
	rm -rf .log
	docker rmi --force "$(IMAGE)" > /dev/null 2>&1 || true

.PHONY: start
start: ## Start LTTng Scope
	@echo "Starting LTTng Scope..."
	mkdir -p $(TRACING_LOG_DIRECTORY)
	docker stop -f $(PROJECT) > /dev/null 2>&1 || true
	docker rm   -f $(PROJECT) > /dev/null 2>&1 || true
	xhost +local:docker
	docker run \
	    -d \
	    --rm \
	    -e DISPLAY=$$DISPLAY \
	    -v $(TRACING_LOG_DIRECTORY):/opt/lttng-scope/tracing_logs \
	    -v /tmp/.X11-unix:/tmp/.X11-unix \
	    --network host \
	    --name $(PROJECT) $(IMAGE)
	@echo "LTTng Scope started..."

.PHONY: stop
stop: ## Stop LTTng Scope
	@echo "Stopping LTTng Scope..."
	docker stop -f $(PROJECT) > /dev/null 2>&1 || true
	docker rm   -f $(PROJECT) > /dev/null 2>&1 || true
	@echo "LTTng Scope stopped"
