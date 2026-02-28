set(CMAKE_SYSTEM_NAME Linux)

# ⚠️ ATTENTION ICI :
# Si l'OS de ta Raspberry Pi est en 64-bits (Ubuntu Pi, Pi OS 64) -> aarch64
# Si c'est le Raspberry Pi OS classique (32-bits) -> arm
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Définition du préfixe (assure-toi que c'est le bon !)
# 64-bits : aarch64-linux-gnu
# 32-bits : arm-linux-gnueabihf
set(CROSS_COMPILE_PREFIX aarch64-linux-gnu)

# Spécification des compilateurs
set(CMAKE_C_COMPILER ${CROSS_COMPILE_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE_PREFIX}-g++)

# --- LE FIX EST LÀ ---
# On indique EXACTEMENT où se trouve l'environnement de la Pi sur ton PC.
# Sur Arch/CachyOS, les paquets de cross-compilation s'installent ici :
set(CMAKE_SYSROOT /usr/${CROSS_COMPILE_PREFIX})
set(CMAKE_FIND_ROOT_PATH /usr/${CROSS_COMPILE_PREFIX})

# Les règles strictes (maintenant elles vont marcher car FIND_ROOT_PATH est défini)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Définition pour ton code C++
add_definitions(-D__CROSS_COMPILE_ARM__)
