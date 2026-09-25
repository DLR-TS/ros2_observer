FROM debian:bookworm-slim AS ros_observer_builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-setuptools \
    dh-python \
    devscripts \
    fakeroot \
    build-essential \
    dh-virtualenv \
    python3-all \
    python3-stdeb \
    python3-yaml \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY . /app

RUN python3 setup.py install 
RUN python3 setup.py --command-packages=stdeb.command bdist_deb

RUN mkdir -p /build/ && mv deb_dist/*.deb /build/

WORKDIR /output

VOLUME ["/output"]

FROM alpine:3.22 AS ros_observer_release
COPY --from=ros_observer_builder /build /build

