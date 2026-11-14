#!/usr/bin/env bash
#
# clangd running inside the x86_64 build image.
#
# This project is compiled entirely inside Docker (see build.sh): OpenCV,
# SQLite, the robot interface repositories under /opt/cdfr and the FetchContent
# dependencies only exist in the image, never on the host. A clangd started on
# the host can therefore never resolve them, however good the compilation
# database is.
#
# This wrapper forwards every invocation to the image, with the repository
# mounted at its own path so the paths recorded in the compilation database
# stay valid. The database is produced (inside the image) in .cache/clangd-db.
#
# Used by the vscode-clangd extension (.vscode/settings.json), and usable from
# a shell, e.g.:  ./scripts/clangd-docker.sh --check=src/app/main.cpp

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
IMAGE="${CDFR_CLANGD_IMAGE:-cdfr-builder-x86_64}"
DOCKERFILE="$REPO_ROOT/docker/Dockerfile.x86_64"
# Dedicated configure directory, kept apart from build/x86_64: that tree is
# wiped by `./build.sh clean` and can be reconfigured on the host by the CMake
# Tools extension, which would record host compiler and include paths clangd
# cannot use.
DB_DIR="$REPO_ROOT/.cache/clangd-db"
DB="$DB_DIR/compile_commands.json"

log() { printf 'clangd-docker: %s\n' "$*" >&2; }

# Inside the image (dev container, ./build.sh shell, ...) clangd is already
# available: never nest containers.
if [ -f /.dockerenv ]; then
    if command -v clangd >/dev/null 2>&1; then
        exec clangd "$@"
    fi
    log "clangd is missing from this image, rebuild it with ./build.sh images"
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    log "the Docker daemon is not reachable, start Docker and restart clangd"
    exit 1
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    log "image $IMAGE not found, building it (first run only)"
    docker build -t "$IMAGE" -f "$DOCKERFILE" "$REPO_ROOT" >&2
fi

# CMake regenerates compile_commands.json on every configure. Refresh it when it
# is missing or older than the build description, so clangd always sees the
# current source list and compiler flags.
needs_configure() {
    [ -f "$DB" ] || return 0
    [ "$REPO_ROOT/CMakePresets.json" -nt "$DB" ] && return 0
    [ -n "$(find "$REPO_ROOT" \
              \( -name build -o -name .cache -o -name .git \) -prune -o \
              -name CMakeLists.txt -newer "$DB" -print -quit)" ] && return 0
    return 1
}

if needs_configure; then
    log "refreshing the compilation database in ${DB_DIR#"$REPO_ROOT"/}"
    # CDFR_COMPILE_COMMANDS_SYMLINK=OFF: this database is clangd's private copy
    # and must not rewrite the ./build.sh symlink at the repository root.
    docker run --rm \
        --security-opt label=disable \
        --user "$(id -u):$(id -g)" \
        -e HOME=/tmp \
        -v "$REPO_ROOT:$REPO_ROOT" \
        -w "$REPO_ROOT" \
        "$IMAGE" cmake --preset x86_64 \
        -B "$DB_DIR" \
        -DCDFR_COMPILE_COMMANDS_SYMLINK=OFF >&2 \
        || log "configure failed, clangd starts without an up-to-date database"
fi

# stdin/stdout carry the LSP stream, hence -i and no -t.
exec docker run --rm -i \
    --security-opt label=disable \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -v "$REPO_ROOT:$REPO_ROOT" \
    -w "$REPO_ROOT" \
    "$IMAGE" clangd "$@"
