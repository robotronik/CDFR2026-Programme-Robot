FROM ubuntu:24.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install host tools, cross-compiler, CMake, Ninja, and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    cmake \
    ninja-build \
    ccache \
    rsync \
    openssh-client \
    git \
    ca-certificates \
    python3 \
    gawk \
    libsqlite3-dev \
    && rm -rf /var/lib/apt/lists/*

# Create non-root builder user
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g ${GROUP_ID} builder 2>/dev/null || true && \
    useradd -u ${USER_ID} -g ${GROUP_ID} -m -s /bin/bash builder 2>/dev/null || true && \
    mkdir -p /home/builder/.cache/ccache && \
    chown -R ${USER_ID}:${GROUP_ID} /home/builder

ENV CCACHE_DIR=/home/builder/.cache/ccache
ENV HOME=/home/builder
ENV IS_CONTAINER=1

WORKDIR /workspace/CDFR2026-Programme-Robot
