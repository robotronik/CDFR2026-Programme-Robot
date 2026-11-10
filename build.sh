#!/usr/bin/env bash
#
# Docker-only build wrapper: every compilation happens inside the
# per-architecture images, so no compiler is needed on the host. The workspace is
# mounted at its own path, so build/x86_64 and build/arm64 appear on the host.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

IMAGE_X86="cdfr-builder-x86_64"
IMAGE_ARM="cdfr-builder-arm64"

# Extra `docker run` flags, user and workdir applied to the next docker_run call;
# callers override these before invoking it (see the run-docker command).
DOCKER_RUN_EXTRA=(--network host)
DOCKER_RUN_USER="$(id -u):$(id -g)"
DOCKER_RUN_WORKDIR="$REPO_ROOT"

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
    local tty=() ssh=() user=()
    [ -t 0 ] && tty+=(-it)
    # An empty DOCKER_RUN_USER keeps the image's default user (root), which is
    # required to actually exercise added capabilities (see run-docker).
    if [ -n "$DOCKER_RUN_USER" ]; then
        user=(--user "$DOCKER_RUN_USER")
    fi
    if [ -n "${SSH_AUTH_SOCK:-}" ] && [ -S "${SSH_AUTH_SOCK}" ]; then
        ssh+=(-v "${SSH_AUTH_SOCK}:/ssh-agent" -e SSH_AUTH_SOCK=/ssh-agent)
    fi
    [ -d "$HOME/.ssh" ] && ssh+=(-v "$HOME/.ssh:/home/ubuntu/.ssh:ro")
    # label=disable: on SELinux hosts the bind mounts are otherwise denied
    # ("Permission denied", which cmake reports as a missing CMakePresets.json).
    # No-op on hosts without SELinux.
    docker run --rm "${tty[@]}" "${ssh[@]}" \
        "${DOCKER_RUN_EXTRA[@]}" \
        --security-opt label=disable \
        "${user[@]}" \
        -v "$REPO_ROOT:$REPO_ROOT" \
        -w "$DOCKER_RUN_WORKDIR" \
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
  run                    Build x86_64 and run the local REST API and simulator
  run-docker [args...]   Build x86_64 and run inside the x86_64 image (no host deps)
  test                   Build x86_64 and run the CTest suite
  deploy                 Build arm64 and deploy it to the robot
  shell [x86_64|arm64]   Interactive shell in the target image (default: x86_64)
  images                 (Re)build both Docker images
  clean                  Remove the build/ directory

Environment:
  CDFR_RUN_PORT          Host port published for the local REST API (default: 80)
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
        DOCKER_RUN_EXTRA=(--tmpfs "$REPO_ROOT/build/x86_64/log:mode=1777")
        docker_run "$IMAGE_X86" sh -ec 'echo 0 > build/x86_64/log/numLog; exec ctest --preset x86_64'
        ;;
    run)
        if (( $# != 1 )); then echo 'Usage: ./build.sh run' >&2; exit 1; fi
        build_arch x86_64
        port=${CDFR_RUN_PORT:-80}
        log "Running the local REST API and simulator at http://127.0.0.1:$port/simulation/"
        DOCKER_RUN_WORKDIR="$REPO_ROOT/build/x86_64"
        DOCKER_RUN_EXTRA=(-p "127.0.0.1:$port:8080" --tmpfs "$DOCKER_RUN_WORKDIR/log:mode=1777")
        # The logger expects numLog in the mounted log directory.
        docker_run "$IMAGE_X86" sh -ec 'echo 0 > log/numLog; exec ./programCDFR --simulate'
        ;;
    run-docker)
        build_arch x86_64
        shift
        log "Running programCDFR in the x86_64 image (no host libraries needed)"
        # setProgramPriority() needs CAP_SYS_NICE, dropped by Docker by default,
        # and the capability only works for uid 0, so run as root (no --user).
        # Files the run writes (log/) are then root-owned on the host.
        DOCKER_RUN_EXTRA=(--network host --cap-add=SYS_NICE)
        DOCKER_RUN_USER=""
        DOCKER_RUN_WORKDIR="$REPO_ROOT/build/x86_64"
        docker_run "$IMAGE_X86" ./programCDFR "$@"
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
