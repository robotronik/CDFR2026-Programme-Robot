#!/usr/bin/env bash
#
# Docker-only build wrapper.
#
# Every compilation happens inside the per-architecture images; no compiler is
# needed on the host. The workspace is mounted at its own path so build
# artifacts (build/x86_64, build/arm64) appear directly on the host.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

IMAGE_X86="cdfr-builder-x86_64"
IMAGE_ARM="cdfr-builder-arm64"

log() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }

build_image() {
    local image="$1" dockerfile="$2"
    log "Building image $image"
    docker build -t "$image" -f "$dockerfile" .
}

ensure_image() {
    local image="$1" dockerfile="$2"
    docker image inspect "$image" >/dev/null 2>&1 || build_image "$image" "$dockerfile"
}

# docker_run <image> <command...>
docker_run() {
    local image="$1"; shift
    local tty=() ssh=()
    [ -t 0 ] && tty+=(-it)
    if [ -n "${SSH_AUTH_SOCK:-}" ] && [ -S "${SSH_AUTH_SOCK}" ]; then
        ssh+=(-v "${SSH_AUTH_SOCK}:/ssh-agent" -e SSH_AUTH_SOCK=/ssh-agent)
    fi
    [ -d "$HOME/.ssh" ] && ssh+=(-v "$HOME/.ssh:/home/ubuntu/.ssh:ro")
    docker run --rm "${tty[@]}" "${ssh[@]}" \
        --network host \
        --user "$(id -u):$(id -g)" \
        -v "$REPO_ROOT:$REPO_ROOT" \
        -w "$REPO_ROOT" \
        "$image" "$@"
}

# build_arch <x86_64|arm64>
build_arch() {
    local arch="$1" image dockerfile
    case "$arch" in
        x86_64) image="$IMAGE_X86"; dockerfile="docker/Dockerfile.x86_64" ;;
        arm64)  image="$IMAGE_ARM"; dockerfile="docker/Dockerfile.arm64" ;;
        *) echo "Unknown architecture: $arch (expected x86_64 or arm64)" >&2; exit 1 ;;
    esac
    ensure_image "$image" "$dockerfile"
    log "Building $arch"
    docker_run "$image" cmake --preset "$arch"
    docker_run "$image" cmake --build --preset "$arch"
}

usage() {
    cat <<EOF
Usage: ./build.sh <command> [arch]

Commands:
  build [x86_64|arm64]   Build the given target, or both when omitted
  run [args...]          Build x86_64 and run the program locally (needs sudo)
  test                   Build x86_64 and run the CTest suite
  deploy                 Build arm64 and deploy it to the robot
  shell [x86_64|arm64]   Interactive shell in the target image (default: x86_64)
  images                 (Re)build both Docker images
  clean                  Remove the build/ directory
EOF
}

case "${1:-build}" in
    build)
        if [ -n "${2:-}" ]; then
            build_arch "$2"
        else
            build_arch x86_64
            build_arch arm64
        fi
        ;;
    test)
        build_arch x86_64
        log "Running tests"
        docker_run "$IMAGE_X86" ctest --preset x86_64
        ;;
    run)
        build_arch x86_64
        shift
        log "Running programCDFR from build/x86_64 (sudo is required for port 80)"
        (cd "$REPO_ROOT/build/x86_64" && sudo ./programCDFR "$@")
        ;;
    deploy)
        ensure_image "$IMAGE_ARM" docker/Dockerfile.arm64
        log "Deploying to robot"
        docker_run "$IMAGE_ARM" cmake --preset arm64
        docker_run "$IMAGE_ARM" cmake --build --preset arm64 --target deploy
        ;;
    shell)
        case "${2:-x86_64}" in
            x86_64) ensure_image "$IMAGE_X86" docker/Dockerfile.x86_64; docker_run "$IMAGE_X86" bash ;;
            arm64)  ensure_image "$IMAGE_ARM" docker/Dockerfile.arm64; docker_run "$IMAGE_ARM" bash ;;
            *) echo "Unknown architecture: $2" >&2; exit 1 ;;
        esac
        ;;
    images)
        build_image "$IMAGE_X86" docker/Dockerfile.x86_64
        build_image "$IMAGE_ARM" docker/Dockerfile.arm64
        ;;
    clean)
        rm -rf build compile_commands.json
        log "Build artifacts removed"
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage
        exit 1
        ;;
esac
