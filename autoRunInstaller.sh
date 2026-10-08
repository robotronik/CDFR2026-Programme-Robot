#!/bin/bash

install_service() {
    # Vérifie si l'utilisateur a fourni un argument
    if [ $# -ne 1 ]; then
        echo "Utilisation: $0 --install /chemin/vers/votre/programme"
        exit 1
    fi

    # Récupère le chemin absolu du programme
    program_path=$(realpath "$1")

    # Vérifie si le fichier binaire existe
    if [ ! -x "$program_path" ]; then
        echo "Le fichier binaire spécifié n'existe pas ou n'est pas exécutable."
        exit 1
    fi

    # Récupère le nom du fichier binaire
    program_name=$(basename "$program_path")

    # Get the directory where this script (and your program) is located
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"


    # Crée le contenu du fichier de service
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

    # Chemin du fichier de service
    service_file="/etc/systemd/system/${program_name}.service"

    # Vérifie si le service existe déjà
    if [ -f "$service_file" ]; then
        echo "Le service existe déjà : $service_file . Il sera redémmaré"
        sudo systemctl restart "${program_name}.service"
        exit 0
    fi

    # Écrit le contenu dans le fichier de service
    sudo touch "$service_file"
    echo "$service_content" | sudo tee -a "$service_file" > /dev/null

    # Recharge systemd pour prendre en compte les modifications
    sudo systemctl daemon-reload

    # Active le service pour qu'il démarre automatiquement au démarrage
    sudo systemctl enable "${program_name}.service"

    sudo systemctl start "${program_name}.service"

    echo "Le service a été installé avec succès : $service_file"
}

uninstall_service() {
    # Vérifie si l'utilisateur a fourni un argument
    if [ $# -ne 1 ]; then
        echo "Utilisation: $0 --uninstall /chemin/vers/votre/programme"
        exit 1
    fi

    # Récupère le nom du fichier binaire
    program_path=$(realpath "$1")
    program_name=$(basename "$program_path")

    # Chemin du fichier de service
    service_file="/etc/systemd/system/${program_name}.service"

    # Vérifie si le service existe : rien à faire (succès) s'il est déjà absent,
    # pour que le déploiement reste fonctionnel sur un robot neuf.
    if [ ! -f "$service_file" ]; then
        echo "Le service n'existe pas : $service_file (rien à désinstaller)"
        exit 0
    fi

    # Désactive et supprime le service
    sudo systemctl stop "${program_name}.service"
    sudo systemctl disable "${program_name}.service"
    sudo rm -f "$service_file"
    sudo systemctl daemon-reload

    echo "Le service a été désinstallé avec succès : $service_file"
}

# Vérifie si une option est fournie
if [ $# -eq 0 ]; then
    echo "Utilisation: $0 [--install/--uninstall] /chemin/vers/votre/programme"
    exit 1
fi

# Récupère l'option fournie
option=$1
shift

# Exécute la fonction correspondante à l'option
case $option in
    "--install") install_service "$@" ;;
    "--uninstall") uninstall_service "$@" ;;
    *) echo "Option non valide. Utilisez --install ou --uninstall." ;;
esac
