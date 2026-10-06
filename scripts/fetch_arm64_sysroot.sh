#!/bin/bash
# Récupère les bibliothèques ARM64 (OpenCV, SQLite, libcamera) nécessaires à la
# cross-compilation, sans passer par l'installation dpkg multiarch.
#
# Pourquoi pas "apt install libopencv-dev:arm64" ? Parce que libopencv-dev n'est
# pas Multi-Arch: same : dpkg refuserait d'installer la variante arm64 à côté de
# la variante amd64, qui elle est indispensable au build local. On se contente
# donc de télécharger les .deb arm64 et de les extraire dans un sysroot local,
# que pi_toolchain.cmake ajoute à CMAKE_FIND_ROOT_PATH via ARM64_SYSROOT.
#
# libcamera est la bibliothèque de capture native de la Raspberry Pi 5 (le
# backend V4L2 n'expose que du Bayer brut derrière l'ISP PiSP). Elle est
# requise par src/vision/LibcameraCamera.cpp pour le build ARM.
set -e

DEST="${1:-$HOME/aarch64-sysroot}"

for tool in apt-get dpkg-deb; do
    command -v "$tool" >/dev/null || { echo "Outil manquant : $tool"; exit 1; }
done

# libcamera-dev : en-têtes + libcamera.so + libcamera.pc.
# libcamera0.x  : la bibliothèque partagée (libcamera.so.0.x) ; le nom du paquet
# suit la version d'API exposée par la distro (libcamera0.2 sur Ubuntu Noble,
# 0.3 sur Raspberry Pi OS Bookworm récent). Le filtre `^libcamera` attrape les
# deux.
PKGS=$(apt-get -s install libopencv-dev:arm64 libsqlite3-dev:arm64 libcamera-dev:arm64 2>/dev/null \
    | awk '/^Inst /{print $2}' \
    | grep -E '^(libopencv|libsqlite3|libcamera)' | grep -vE 'jni|java')


if [ -z "$PKGS" ]; then
    echo "Impossible de résoudre les paquets arm64." >&2
    echo "Vérifiez que l'architecture arm64 est déclarée, par exemple :" >&2
    echo "  /etc/apt/sources.list.d/arm64.sources" >&2
    echo "    Types: deb" >&2
    echo "    URIs: http://ports.ubuntu.com/ubuntu-ports" >&2
    echo "    Suites: noble noble-updates noble-security" >&2
    echo "    Components: main universe" >&2
    echo "    Architectures: arm64" >&2
    echo "puis 'sudo apt update'." >&2
    exit 1
fi

echo "$(echo "$PKGS" | wc -l) paquets arm64 à récupérer :"
echo "$PKGS" | tr '\n' ' ' | fold -sw 100 | sed 's/^/  /'

rm -rf "$DEST/debs" "$DEST/root"
mkdir -p "$DEST/debs" "$DEST/root"

(cd "$DEST/debs" && apt-get download $PKGS)
for d in "$DEST"/debs/*.deb; do dpkg-deb -x "$d" "$DEST/root"; done

echo
echo "Sysroot prêt : $DEST/root"
echo "Build        : ARM64_SYSROOT=$DEST/root ./build.sh build_arm"
