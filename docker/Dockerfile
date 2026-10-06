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
    pkg-config \
    rsync \
    openssh-client \
    git \
    ca-certificates \
    python3 \
    gawk \
    && rm -rf /var/lib/apt/lists/*

# ARM64 cross sysroot: OpenCV, SQLite and libcamera cannot be installed as
# `:arm64` next to their amd64 counterparts (libopencv-dev is not
# Multi-Arch:same). The installer resolver also refuses to plan a foreign-arch
# libc, so the dependency closure is read from the cache (apt-cache depends) and
# each .deb is downloaded and extracted into a sysroot.
# cmake/aarch64_toolchain.cmake points at it through SYSROOT for the ARM preset
# (the cross toolchain already provides the arm64 C library). The amd64 -dev
# libraries are installed afterwards so they cannot interfere.
RUN set -eux; \
    dpkg --add-architecture arm64; \
    sed -i '/^Types: deb$/a Architectures: amd64' /etc/apt/sources.list.d/ubuntu.sources; \
    printf 'deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports noble main universe\n' \
           'deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports noble-updates main universe\n' \
           'deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports noble-security main universe\n' \
           > /etc/apt/sources.list.d/arm64.list; \
    apt-get update; \
    mkdir -p /opt/arm64-sysroot /tmp/arm64-debs; \
    cd /tmp/arm64-debs; \
    { apt-cache depends --recurse --no-recommends --no-suggests --no-conflicts \
          --no-breaks --no-replaces --no-enhances \
          libopencv-dev:arm64 libsqlite3-dev:arm64 libcamera-dev:arm64 2>/dev/null \
        | sed -nE 's/^[[:space:]]*\|?(Pre)?Depends:[[:space:]]*([^[:space:]]+).*/\2/p'; \
      echo libopencv-dev:arm64; echo libsqlite3-dev:arm64; echo libcamera-dev:arm64; } \
      | grep -E '^(libopencv|libsqlite3|libcamera)' | grep -vE 'jni|java' | sort -u > /tmp/arm64-pkgs.txt; \
    test -s /tmp/arm64-pkgs.txt; \
    while read -r pkg; do apt-get download "$pkg"; done < /tmp/arm64-pkgs.txt; \
    test "$(ls /tmp/arm64-debs/*.deb | wc -l)" -gt 0; \
    for deb in /tmp/arm64-debs/*.deb; do dpkg-deb -x "$deb" /opt/arm64-sysroot; done; \
    rm -rf /tmp/arm64-debs /tmp/arm64-pkgs.txt
ENV SYSROOT=/opt/arm64-sysroot

# amd64 development libraries for the local (x86_64) build inside the container
RUN apt-get update && apt-get install -y --no-install-recommends \
    libsqlite3-dev \
    libopencv-dev \
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
