#!/bin/bash

# --- Config Raspi ---
PI_USER="robotronik"; PI_HOST="raspitronik.local"; PI_DIR="/home/$PI_USER/CDFR"; PI_DEST="arm_bin"

# --- Config build ---
GEN=""; OPT=""
LIDAR_LIB="rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a"

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

# --- Fonctions de Build ---

# Helper d'affichage compact [BG_COLOR] [FG_COLOR] [LABEL] [MESSAGE]
# Format [BADGE = BG_COLOR + LABEL] [MESSAGE = FG_COLOR + TEXT]
 
step() { printf "${1}${BOLD} %-10s ${NC} ${2}%s${NC}\n" "$3" "$4"; }

# Helpers de build

setup_generator() {
    if command -v ninja >/dev/null 2>&1; then
        step "$BG_BLU" "$F_BLU" "CONFIG" "Ninja détecté."
        GEN="-GNinja"
        OPT=""
    else
        step "$BG_ORG" "$F_ORG" "WARN" "Make utilisé (Ninja absent)."
        GEN="-GUnix Makefiles"
        OPT="-- -j$(nproc)"
    fi
}

check_lidar_arch() {
    local target=$(echo "$1" | tr '[:lower:]' '[:upper:]')
    if [ ! -f "$LIDAR_LIB" ]; then return 1; fi
    local machine_line=$(readelf -h "$LIDAR_LIB" 2>/dev/null | grep "Machine:" | head -n 1 | tr '[:lower:]' '[:upper:]')
    if [[ "$machine_line" == *"$target"* ]]; then
        return 0
    fi
    return 1
}

# Compilation complète (génération cmake + build)

build_lidar() { 
    if check_lidar_arch "X86-64"; then
        step "$BG_GRN" "$F_GRN" "SKIP" "SDK RPLidar (x86_64) déjà à jour."
    else
        step "$BG_BLU" "$F_BLU" "INFO" "Compilation SDK RPLidar (x86_64)..."
        (cd rplidar_sdk && make clean >/dev/null 2>&1 && make -s >/dev/null)
        step "$BG_GRN" "$F_GRN" "DONE" "SDK RPLidar (Local) prêt."
    fi
}

build_lidar_arm() {  
    if check_lidar_arch "AArch64"; then
        step "$BG_GRN" "$F_GRN" "SKIP" "SDK RPLidar (ARM64) déjà à jour."
    else
        step "$BG_BLU" "$F_BLU" "INFO" "Compilation SDK RPLidar (ARM)..."
        (cd rplidar_sdk && make clean >/dev/null 2>&1 && \
         make CC=aarch64-linux-gnu-gcc CXX=aarch64-linux-gnu-g++ AR=aarch64-linux-gnu-ar -s >/dev/null)
        step "$BG_GRN" "$F_GRN" "DONE" "SDK RPLidar (ARM) prêt."
    fi
}

build_local() {
    setup_generator
    build_lidar
    step "$BG_BLU" "$F_BLU" "BUILD" "Compilation locale (x86_64)..."
    cmake $GEN -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-fdiagnostics-color=always" >/dev/null
    cmake --build build $OPT
    step "$BG_GRN" "$F_GRN" "DONE" "Binaire compilé."
}

build_arm() {
    setup_generator
    build_lidar_arm 
    step "$BG_BLU" "$F_BLU" "BUILD" "Cross-compilation ARM64..."
    cmake $GEN -B build_arm -DCMAKE_TOOLCHAIN_FILE=pi_toolchain.cmake >/dev/null
    cmake --build build_arm $OPT
    step "$BG_GRN" "$F_GRN" "DONE" "Binaire ARM compilé."
}

# Fonction pour setup LSP (compile_commands.json et link)
setup_lsp() {
    step "$BG_BLU" "$F_BLU" "SETUP" "Génération LSP..."
    cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >/dev/null
    [ -f "build/compile_commands.json" ] && { ln -sf build/compile_commands.json .; step "$BG_GRN" "$F_GRN" "DONE" "Lien créé."; } \
                                         || step "$BG_RED" "$F_RED" "ERROR" "Échec LSP."
}

# Sync sur la pi
deploy_pi() {
    run_timed "Build ARM" build_arm

    step "$BG_BLU" "$F_BLU" "SYNC" "Transfert vers le robot ($PI_HOST)..."
    ssh $PI_USER@$PI_HOST "mkdir -p $PI_DIR/$PI_DEST"
    rsync -az --progress --delete ./build_arm/data ./build_arm/html autoRunInstaller.sh ./build_arm/programCDFR ./build_arm/pi_detect_aruco.py $PI_USER@$PI_HOST:$PI_DIR/$PI_DEST | grep -v "/$"
    rsync_status=${PIPESTATUS[0]}
    if [ "$rsync_status" -ne 0 ]; then
        step "$BG_RED" "$F_RED" "ERROR" "Échec du transfert (rsync)."
        return "$rsync_status"
    fi    
    scp -q autoRunInstaller.sh $PI_USER@$PI_HOST:$PI_DIR/$PI_DEST

    step "$BG_BLU" "$F_BLU" "SERVICE" "Installation et démarrage auto..."
    ssh -t $PI_USER@$PI_HOST "cd $PI_DIR/$PI_DEST && chmod +x autoRunInstaller.sh && \
        sudo ./autoRunInstaller.sh --uninstall $PI_DIR/$PI_DEST/programCDFR && \
        sudo ./autoRunInstaller.sh --install $PI_DIR/$PI_DEST/programCDFR"
    
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
    local w_cnt=$(echo "$clean" | grep -icE "warning:|\[WARNING\]")
    local e_cnt=$(echo "$clean" | grep -icE "error:|\[ERROR\]")
    
    # Formatage dynamique des couleurs pour les stats
    local w_col="$F_GRN"; [ "$w_cnt" -gt 0 ] && w_col="$F_ORG"
    local e_col="$F_GRN"; [ "$e_cnt" -gt 0 ] && e_col="$F_RED"
    local c_base="$F_GRN"; [ $st -ne 0 ] && c_base="$F_RED"
    
    local stats=" | ${w_col}${w_cnt} Warn(s)${c_base} | ${e_col}${e_cnt} Err(s)${c_base}"
    
    [ $st -eq 0 ] && [ $e_cnt -eq 0 ] && step "$BG_GRN" "$F_GRN" "SUCCESS" "$task terminé en ${dur}s${stats}" \
                  || { step "$BG_RED" "$F_RED" "FAIL" "$task échoué en ${dur}s${stats}"; }
}

clean() {
    if [ -d "build" ]; then
        cmake --build build --target clean
        step "$BG_GRN" "$F_GRN" "DONE" "Build local nettoyé."
    else
        step "$BG_BLU" "$F_BLU" "INFO" "Pas de build local."
    fi
    
    if [ -d "build_arm" ]; then
        cmake --build build_arm --target clean
        step "$BG_GRN" "$F_GRN" "DONE" "Build ARM nettoyé." 
    else
        step "$BG_BLU" "$F_BLU" "INFO" "Pas de build ARM."
    fi

    step "$BG_GRN" "$F_GRN" "CLEAN" "Fichiers de build nettoyés."
}

# --- Menu ---

case "$1" in
    build)        run_timed "Build Local" build_local ;;
    build_arm)    run_timed "Build ARM" build_arm ;;
    deploy)    
               run_timed "Déploiement Complet" deploy_pi 
               echo -e "${WHT}--------------------------------------------------------${NC}"
               step "$BG_BLU" "$F_BLU" "LOGS" "En direct du robot (Ctrl+C pour quitter l'affichage)..." 
               ssh -t $PI_USER@$PI_HOST "journalctl -u programCDFR -f --output=cat"
               ;;
    setup-lsp)    run_timed "Setup LSP" setup_lsp ;;
    tests)        run_timed "Build local" build_local; \
                   if [ -f "build/robot_tests" ]; then \
                        [ -e "lidar" ] || ln -s "tests/lidar" "lidar"; \
                        run_timed "Tests" ./build/robot_tests; \
                        rm "lidar"; \
                   else \
                       step "$BG_RED" "$F_RED" "ERROR" "Test introuvable"; \
                   fi ;;
    clean-all)    rm -rf build build_arm compile_commands.json; step "$BG_GRN" "$F_GRN" "CLEAN ALL" "Dossiers supprimés." ;;
    clean)        clean ;; 
    *)            echo -e "${BOLD}Usage:${NC} $0 {build|build_arm|deploy|setup-lsp|tests|clean|clean-all}"; exit 1 ;;
esac
