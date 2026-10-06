#!/bin/bash

# Palette de couleurs
export CLICOLOR_FORCE=1
export FORCE_COLOR=1
ESC=$'\033'
NC="${ESC}[0m"; BOLD="${ESC}[1m"; WHT="${ESC}[37m"
F_GRN="${ESC}[38;5;107m"; BG_GRN="${ESC}[30;48;5;107m"
F_RED="${ESC}[38;5;124m"; BG_RED="${ESC}[30;48;5;124m"
F_BLU="${ESC}[38;5;72m";  BG_BLU="${ESC}[30;48;5;72m"

step() { printf "${1}${BOLD} %-10s ${NC} ${2}%s${NC}\n" "$3" "$4"; }

# Docker n'est pas forcément installé sur la machine hôte ; podman expose la même
# interface en ligne de commande. On l'utilise comme repli pour toutes les
# commandes `docker` (build, run, image inspect).
# Deux ajustements pour `run` sous podman rootless :
#   --userns=keep-id  : le conteneur doit pouvoir écrire dans les sources
#                       montées avec l'uid de l'hôte (cf. --user dans docker_run) ;
#   label=disable     : les montages de l'hôte restent lisibles sous SELinux.
if ! command -v docker >/dev/null 2>&1 && command -v podman >/dev/null 2>&1; then
    docker() {
        if [ "$1" = "run" ]; then
            shift
            podman run --userns=keep-id --security-opt label=disable "$@"
        else
            podman "$@"
        fi
    }
fi

run_timed() {
    local task="$1"; shift
    local t0=$(date +%s.%N)

    step "$BG_BLU" "$F_BLU" "EXEC" "${BOLD}$task"
    echo "--------------------------------------------------------"

    "$@"
    local st=$?

    local t1=$(date +%s.%N)
    local dur=$(awk -v t0="$t0" -v t1="$t1" 'BEGIN {printf "%.2f", t1 - t0}')
    echo "--------------------------------------------------------"

    if [ $st -eq 0 ]; then
        step "$BG_GRN" "$F_GRN" "SUCCESS" "$task completed in ${dur}s"
    else
        step "$BG_RED" "$F_RED" "FAIL" "$task failed in ${dur}s"
        exit 1
    fi
}

docker_run() {
    local img="cdfr-builder:latest"

    # Vérification ou construction de l'image
    if ! docker image inspect "$img" >/dev/null 2>&1; then
        step "$BG_BLU" "$F_BLU" "DOCKER" "Construction de l'image Docker ($img)..."
        docker build -t "$img" -f docker/Dockerfile .
        if [ $? -ne 0 ]; then
            step "$BG_RED" "$F_RED" "ERROR" "Échec de la construction Docker."
            exit 1
        fi
    fi

    local ws_root="$(realpath ..)"
    local ccache_host="$HOME/.cache/cdfr-docker-ccache"
    mkdir -p "$ccache_host"

    local extra_args=()
    if [ -t 0 ]; then
        extra_args+=(-it)
    fi

    # Forwarding de l'agent SSH pour deploy si présent
    if [ -n "$SSH_AUTH_SOCK" ] && [ -S "$SSH_AUTH_SOCK" ]; then
        extra_args+=(-v "$SSH_AUTH_SOCK:/ssh-agent" -e "SSH_AUTH_SOCK=/ssh-agent")
    fi
    if [ -d "$HOME/.ssh" ]; then
        extra_args+=(-v "$HOME/.ssh:/home/builder/.ssh:ro")
    fi

    # Détection conteneur vs hôte pour isoler les dossiers de build
    local is_container_arg=()
    is_container_arg+=(-e "IS_CONTAINER=1")

    step "$BG_BLU" "$F_BLU" "DOCKER" "Exécution dans le conteneur: $*"
    docker run --rm \
        "${extra_args[@]}" \
        "${is_container_arg[@]}" \
        --network host \
        --user "$(id -u):$(id -g)" \
        -v "$ws_root:$ws_root" \
        -v "$ws_root:/workspace" \
        -v "$ccache_host:/home/builder/.cache/ccache" \
        -w "$PWD" \
        -e "CCACHE_DIR=/home/builder/.cache/ccache" \
        -e "HOME=/home/builder" \
        "$img" \
        "$@"
}

# Détection de l'environnement d'exécution (Docker vs Hôte)
if [ -n "$IS_CONTAINER" ] || [ -f /.dockerenv ]; then
    PRESET_LOCAL="docker-local"
    PRESET_ARM="docker-arm"
    BUILD_DIR_LOCAL="build-docker"
    BUILD_DIR_ARM="build_arm-docker"
else
    PRESET_LOCAL="local"
    PRESET_ARM="arm"
    BUILD_DIR_LOCAL="build"
    BUILD_DIR_ARM="build_arm"
fi

case "$1" in
    build)
        [ -f "$BUILD_DIR_LOCAL/build.ninja" ] || run_timed "Config Local" cmake --preset "$PRESET_LOCAL"
        run_timed "Build Local (x86_64)" cmake --build --preset "$PRESET_LOCAL"
        ;;
    build_arm)
        [ -f "$BUILD_DIR_ARM/build.ninja" ] || run_timed "Config ARM" cmake --preset "$PRESET_ARM"
        run_timed "Build ARM (AArch64)" cmake --build --preset "$PRESET_ARM"
        ;;
    tests)
        [ -f "$BUILD_DIR_LOCAL/build.ninja" ] || run_timed "Config Local" cmake --preset "$PRESET_LOCAL"
        run_timed "Build Local" cmake --build --preset "$PRESET_LOCAL"
        run_timed "Tests (CTest)" ctest --preset "$PRESET_LOCAL"
        ;;
    deploy)
        [ -f "$BUILD_DIR_ARM/build.ninja" ] || run_timed "Config ARM" cmake --preset "$PRESET_ARM"
        run_timed "Déploiement Robot" cmake --build --preset "$PRESET_ARM" --target deploy
        ;;
    logs)
        step "$BG_BLU" "$F_BLU" "LOGS" "Journalctl en direct (Ctrl+C pour quitter)..."
        cmake --build --preset "$PRESET_ARM" --target logs
        ;;
    setup-lsp)
        run_timed "Setup LSP" cmake --preset "$PRESET_LOCAL"
        ;;
    clean)
        [ -d "$BUILD_DIR_LOCAL" ] && cmake --build --preset "$PRESET_LOCAL" --target clean
        [ -d "$BUILD_DIR_ARM" ] && cmake --build --preset "$PRESET_ARM" --target clean
        step "$BG_GRN" "$F_GRN" "DONE" "Artefacts nettoyés."
        ;;
    clean-all)
        rm -rf build build_arm build-docker build_arm-docker compile_commands.json
        step "$BG_GRN" "$F_GRN" "CLEAN ALL" "Dossiers de build supprimés."
        ;;
    docker)
        shift
        if [ $# -eq 0 ]; then
            echo -e "${BOLD}Usage:${NC} $0 docker {build|build_arm|tests|deploy|logs|clean|clean-all|build-image|shell}"
            exit 1
        fi
        if [ "$1" = "build-image" ]; then
            step "$BG_BLU" "$F_BLU" "DOCKER" "Reconstruction de l'image Docker..."
            docker build -t "cdfr-builder:latest" -f docker/Dockerfile .
        elif [ "$1" = "shell" ]; then
            shift
            docker_run bash "$@"
        else
            docker_run ./build.sh "$@"
        fi
        ;;
    *)
        echo -e "${BOLD}Usage:${NC} $0 {build|build_arm|tests|deploy|logs|setup-lsp|clean|clean-all|docker}"
        echo -e "Commandes Docker :"
        echo -e "  $0 docker build        # Compile en local dans Docker"
        echo -e "  $0 docker build_arm    # Cross-compile pour ARM dans Docker"
        echo -e "  $0 docker tests        # Lance les tests CTest dans Docker"
        echo -e "  $0 docker shell        # Ouvre un shell interactif dans le conteneur"
        echo -e ""
        echo -e "Ou directement avec CMake sur l'hôte :"
        echo -e "  cmake --build --preset local"
        echo -e "  cmake --build --preset arm"
        echo -e "  ctest --preset local"
        echo -e "  cmake --build --preset arm --target deploy"
        echo -e "  cmake --build --preset arm --target logs"
        exit 1
        ;;
esac
