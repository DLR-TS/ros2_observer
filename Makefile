SHELL := /bin/bash

BUILD_DIR  := .
OUTPUT_DIR := /build

ARCH     ?= $(shell uname -m)
TAG      := $(ARCH)_$(shell git rev-parse --short HEAD 2>/dev/null || echo NOHASH)
REPO     ?= $(shell git config --get remote.origin.url 2>/dev/null \
              | sed -e 's|.*github.com[:/]||' -e 's|\.git$$||' \
              | tr '[:upper:]' '[:lower:]')
REPO_LC  := $(shell echo "$(REPO)" | tr '[:upper:]' '[:lower:]')

DOCKER_IMAGE   := ros2-observer:$(TAG)
REGISTRY_IMAGE := ghcr.io/$(REPO_LC)/ros2-observer:$(TAG)

.PHONY: build
build:
	cd trace_compass && make build ARCH="$(ARCH)" REPO="$(REPO)"
	cd lttng_scope   && make build ARCH="$(ARCH)" REPO="$(REPO)"
	mkdir -p $(BUILD_DIR)
	docker pull "$(REGISTRY_IMAGE)" 2>/dev/null && \
	    docker tag "$(REGISTRY_IMAGE)" "$(DOCKER_IMAGE)" || \
	docker build \
	    --cache-from "$(REGISTRY_IMAGE)" \
	    -t "$(DOCKER_IMAGE)" \
	    .
	docker run --rm -v "$(BUILD_DIR)":"$(OUTPUT_DIR)" "$(DOCKER_IMAGE)"
	docker cp $$(docker create --rm "$(DOCKER_IMAGE)"):"$(OUTPUT_DIR)"/ "$(BUILD_DIR)"

.PHONY: push
push: ## Push all images to ghcr.io
	cd trace_compass && make push ARCH="$(ARCH)" REPO="$(REPO)"
	cd lttng_scope   && make push ARCH="$(ARCH)" REPO="$(REPO)"
	docker tag  "$(DOCKER_IMAGE)" "$(REGISTRY_IMAGE)"
	docker push "$(REGISTRY_IMAGE)"

.PHONY: install
install: _install clean

.PHONY: _install
_install:
	python3 setup.py install

.PHONY: clean
clean:
	cd trace_compass && make clean
	cd lttng_scope   && make clean
	rm -rf ros2tools.egg-info build dist ros2tools/__pycache__
	rm -rf build
	docker rmi "$(DOCKER_IMAGE)" --force || true
