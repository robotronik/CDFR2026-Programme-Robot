#!/bin/bash

# --- Config Raspi ---
PI_USER="robotronik"; PI_HOST="192.168.0.102"; PI_DIR="/home/robotronik/CDFR"

# --- Palette ---
export CLICOLOR_FORCE=1
export FORCE_COLOR=1
ESC=$'\033'
NC="${ESC}[0m"; BOLD="${ESC}[1m"; WHT="${ESC}[37m"
F_GRN="${ESC}[38;5;107m"; BG_GRN="${ESC}[30;48;5;107m" # Succès / Zéro erreur
F_RED="${ESC}[38;5;124m"; BG_RED="${ESC}[30;48;5;124m" # Erreur
F_ORG="${ESC}[38;5;172m"; BG_ORG="${ESC}[30;48;5;172m" # Warning
F_BLU="${ESC}[38;5;72m";  BG_BLU="${ESC}[30;48;5;72m"  # Info / Exec / Build

export NINJA_STATUS="${F_GRN}[%p]${NC} ${F_BLU}[%es]${NC} "

# Helper d'affichage compact
step() { printf "${1}${BOLD} %-10s ${NC} ${2}%s${NC}\n" "$3" "$4"; }

# --- Fonctions de Build ---

build_lidar() {
    step "$BG_BLU" "$F_BLU" "INFO" "Compilation SDK RPLidar (x86_64)..."
    (cd rplidar_sdk && make clean >/dev/null 2>&1 && make -s >/dev/null)
    step "$BG_GRN" "$F_GRN" "DONE" "SDK RPLidar (Local) prêt."
}

build_lidar_arm() { 
    step "$BG_BLU" "$F_BLU" "INFO" "Compilation SDK RPLidar (ARM)..." 
    (cd rplidar_sdk && make clean >/dev/null 2>&1 && make CC=aarch64-linux-gnu-gcc CXX=aarch64-linux-gnu-g++ AR=aarch64-linux-gnu-ar >/dev/null)
    step "$BG_GRN" "$F_GRN" "DONE" "SDK RPLidar (ARM) prêt."
}

build_local() {
    build_lidar
    local GEN=""; local OPT="-- -j$(nproc)"
    command -v ninja >/dev/null 2>&1 && { GEN="-G Ninja"; OPT=""; step "$BG_BLU" "$F_BLU" "CONFIG" "Ninja détecté."; } \
                                     || step "$BG_ORG" "$F_ORG" "WARN" "Make utilisé (Ninja absent)."
    
    step "$BG_BLU" "$F_BLU" "BUILD" "Compilation locale (x86_64)..."
    cmake $GEN -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-fdiagnostics-color=always" >/dev/null
    cmake --build build $OPT
    step "$BG_GRN" "$F_GRN" "DONE" "Binaire compilé."
}

setup_ide() {
    step "$BG_BLU" "$F_BLU" "SETUP" "Génération LSP..."
    cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >/dev/null
    [ -f "build/compile_commands.json" ] && { ln -sf build/compile_commands.json .; step "$BG_GRN" "$F_GRN" "DONE" "Lien créé."; } \
                                         || step "$BG_RED" "$F_RED" "ERROR" "Échec LSP."
}

deploy_pi() {
    build_lidar_arm
    local GEN=""; local OPT="-- -j$(nproc)"
    command -v ninja >/dev/null 2>&1 && { GEN="-G Ninja"; OPT=""; }

    step "$BG_BLU" "$F_BLU" "BUILD" "Cross-compilation ARM64..."
    cmake $GEN -B build_arm -DCMAKE_TOOLCHAIN_FILE=../pi_toolchain.cmake >/dev/null
    cmake --build build_arm $OPT

    step "$BG_BLU" "$F_BLU" "SYNC" "Transfert vers le robot ($PI_HOST)..."
    ssh $PI_USER@$PI_HOST "mkdir -p $PI_DIR"
    rsync -az --delete ./build_arm/ $PI_USER@$PI_HOST:$PI_DIR | grep -v "/$"
    scp -q autoRunInstaller.sh $PI_USER@$PI_HOST:$PI_DIR/

    step "$BG_BLU" "$F_BLU" "SERVICE" "Installation et démarrage auto..."
    ssh -t $PI_USER@$PI_HOST "cd $PI_DIR && chmod +x autoRunInstaller.sh && \
        sudo ./autoRunInstaller.sh --uninstall $PI_DIR/programCDFR >/dev/null 2>&1 || true && \
        sudo ./autoRunInstaller.sh --install $PI_DIR/programCDFR"
    
    step "$BG_GRN" "$F_GRN" "DONE" "Robot flashé et programme lancé !"
}

# --- Wrapper de Temps & Analyseur ---

run_timed() {
    local task="$1"; shift; local t0=$(date +%s.%N); local log="/tmp/cdfr_build.log"
    
    step "$BG_BLU" "$F_BLU" "EXEC" "${BOLD}$task"
    echo -e "${WHT}--------------------------------------------------------${NC}"
    
    # Exécution avec redirection propre
    "$@" 2>&1 | tee "$log"
    local st=${PIPESTATUS[0]}
    
    local dur=$(LC_NUMERIC=C printf "%.2f" $(echo "$(date +%s.%N) - $t0" | bc))
    echo -e "${WHT}--------------------------------------------------------${NC}"
    
    # Comptage des erreurs et warnings
    local clean=$(sed -E 's/\x1B\[[0-9;]*[mK]//g' "$log")
    local w_cnt=$(echo "$clean" | grep -ic "warning:")
    local e_cnt=$(echo "$clean" | grep -ic "error:")
    
    # Formatage dynamique des couleurs pour les stats
    local w_col="$F_GRN"; [ "$w_cnt" -gt 0 ] && w_col="$F_ORG"
    local e_col="$F_GRN"; [ "$e_cnt" -gt 0 ] && e_col="$F_RED"
    local c_base="$F_GRN"; [ $st -ne 0 ] && c_base="$F_RED"
    
    local stats=" | ${w_col}${w_cnt} Warn(s)${c_base} | ${e_col}${e_cnt} Err(s)${c_base}"
    
    [ $st -eq 0 ] && step "$BG_GRN" "$F_GRN" "SUCCESS" "$task terminé en ${dur}s${stats}" \
                  || { step "$BG_RED" "$F_RED" "FAIL" "$task échoué en ${dur}s${stats}"; exit $st; }
}

# --- Menu ---

case "$1" in
    build)        run_timed "Build Local" build_local ;;
    deploy)    
               run_timed "Déploiement Complet" deploy_pi 
               echo -e "${WHT}--------------------------------------------------------${NC}"
               step "$BG_BLU" "$F_BLU" "LOGS" "En direct du robot (Ctrl+C pour quitter l'affichage)..." 
               ssh -t $PI_USER@$PI_HOST "journalctl -u programCDFR.service -f -n 50"
               ;;
    setup-ide)    run_timed "Setup IDE" setup_ide ;;
    tests)        run_timed "Tests Locaux" build_local; [ -f "build/robot_tests" ] && ./build/robot_tests || step "$BG_RED" "$F_RED" "ERROR" "Test introuvable" ;; 
    clean)        rm -rf build build_arm compile_commands.json; step "$BG_GRN" "$F_GRN" "CLEAN" "Dossiers supprimés." ;;
    *)            echo -e "${BOLD}Usage:${NC} $0 {build|deploy|setup-ide|tests|clean}"; exit 1 ;;
esac
