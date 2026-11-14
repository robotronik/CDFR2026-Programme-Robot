#!/bin/bash

# The binary is built against these exact libcamera packages (see
# docker/Dockerfile.arm64); an apt upgrade to another revision breaks the camera.
hold_libcamera() {
    local pkgs=(libcamera0.8 libcamera-ipa libpisp1 libpisp-common)
    local missing=()
    for pkg in "${pkgs[@]}"; do
        dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
    done
    if [ ${#missing[@]} -ne 0 ]; then
        echo "AVERTISSEMENT : paquets libcamera absents, non figés : ${missing[*]}" >&2
        echo "Installez-les avec : sudo apt install ${missing[*]}" >&2
    fi
    sudo apt-mark hold "${pkgs[@]}" >/dev/null
}

install_service() {
    if [ $# -ne 1 ]; then
        echo "Utilisation: $0 --install /chemin/vers/votre/programme"
        exit 1
    fi

    program_path=$(realpath "$1")

    if [ ! -x "$program_path" ]; then
        echo "Le fichier binaire spécifié n'existe pas ou n'est pas exécutable."
        exit 1
    fi

    program_name=$(basename "$program_path")
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

    hold_libcamera

    service_content="[Unit]
Description=Service pour $program_name
After=network.target

[Service]
Type=simple
ExecStart=$program_path
Restart=always
RestartSec=3
User=root
AmbientCapabilities=CAP_SYS_NICE CAP_NET_BIND_SERVICE
StandardOutput=journal
StandardError=journal
WorkingDirectory=$SCRIPT_DIR
[Install]
WantedBy=multi-user.target
"

    service_file="/etc/systemd/system/${program_name}.service"

    if [ -f "$service_file" ]; then
        echo "Le service existe déjà : $service_file . Il sera redémmaré"
        sudo systemctl restart "${program_name}.service"
        exit 0
    fi

    sudo touch "$service_file"
    echo "$service_content" | sudo tee -a "$service_file" > /dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable "${program_name}.service"
    sudo systemctl start "${program_name}.service"

    echo "Le service a été installé avec succès : $service_file"
}

uninstall_service() {
    if [ $# -ne 1 ]; then
        echo "Utilisation: $0 --uninstall /chemin/vers/votre/programme"
        exit 1
    fi

    program_path=$(realpath "$1")
    program_name=$(basename "$program_path")
    service_file="/etc/systemd/system/${program_name}.service"

    # Rien à faire (succès) si le service est déjà absent, pour que le déploiement
    # reste fonctionnel sur un robot neuf.
    if [ ! -f "$service_file" ]; then
        echo "Le service n'existe pas : $service_file (rien à désinstaller)"
        exit 0
    fi

    sudo systemctl stop "${program_name}.service"
    sudo systemctl disable "${program_name}.service"
    sudo rm -f "$service_file"
    sudo systemctl daemon-reload

    echo "Le service a été désinstallé avec succès : $service_file"
}

if [ $# -eq 0 ]; then
    echo "Utilisation: $0 [--install/--uninstall] /chemin/vers/votre/programme"
    exit 1
fi

option=$1
shift

case $option in
    "--install") install_service "$@" ;;
    "--uninstall") uninstall_service "$@" ;;
    *) echo "Option non valide. Utilisez --install ou --uninstall." ;;
esac
