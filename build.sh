#!/bin/bash

# --- Configuration ---
PI_USER="robotronik"
PI_HOST="192.168.0.102"
PI_DIR="/home/robotronik/CDFR"

# Couleurs
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

# --- Fonctions ---

function build_lidar_sdk() {
    if [ ! -f "rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a" ]; then
        echo -e "${BLUE}[INFO] Compilation du SDK RPLidar...${NC}"
        make -C rplidar_sdk
    fi
    echo -e "${GREEN}[DONE] SDK RPLidar prêt.${NC}"
}

function build_lidar_sdk_arm() { 
    echo -e "${BLUE}[INFO] Compilation du SDK RPLidar pour ARM...${NC}"
    cd rplidar_sdk && chmod +x ./cross_compile.sh && ./cross_compile.sh && cd ..
    echo -e "${GREEN}[DONE] SDK RPLidar ARM prêt.${NC}"
}

function build_local() {
    build_lidar_sdk
    echo -e "${BLUE}[INFO] Build local (x86_64)...${NC}"
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    cd ..
}

function run_tests_local() {
    build_local
    if [ -f "build/robot_tests" ]; then
        echo -e "${GREEN}[RUN] Lancement des tests locaux...${NC}"
        ./build/robot_tests
    else
        echo -e "${RED}[ERROR] Exécutable 'robot_tests' introuvable dans build/.${NC}"
    fi
}

function deploy_pi() {
    build_lidar_sdk_arm
    echo -e "${BLUE}[INFO] Build pour Raspberry Pi (ARM64)...${NC}"
    mkdir -p build_arm && cd build_arm
    cmake .. -DCMAKE_TOOLCHAIN_FILE=../pi_toolchain.cmake
    make -j$(nproc)
    cd ..

    echo -e "${BLUE}[INFO] Transfert vers $PI_HOST...${NC}"
    ssh $PI_USER@$PI_HOST "mkdir -p $PI_DIR"
    # rsync synchronise tout le contenu de build_arm (binaires + ressources copiées par CMake)
    rsync -avz --progress ./build_arm/ $PI_USER@$PI_HOST:$PI_DIR
}

function run_tests_remote() {
    deploy_pi
    echo -e "${GREEN}[RUN] Lancement des tests sur la Raspberry Pi...${NC}"
    ssh -t $PI_USER@$PI_HOST "cd $PI_DIR && ./robot_tests"
}

function run_robot() {
    echo -e "${BLUE}[INFO] Lancement du programme principal sur la Pi...${NC}"
    ssh -t $PI_USER@$PI_HOST "cd $PI_DIR && sudo ./programCDFR"
}

# --- Menu ---

case "$1" in
    build)        build_local ;;
    deploy)       deploy_pi ;;
    run)          run_robot ;;
    tests)        run_tests_local ;;
    deploy-tests) run_tests_remote ;;
    clean)
        rm -rf build build_arm
        echo -e "${GREEN}[DONE] Clean terminé${NC}"
        ;;
    *)
        echo "Usage: $0 {build|deploy|run|tests|deploy-tests|clean}"
        exit 1
esac
