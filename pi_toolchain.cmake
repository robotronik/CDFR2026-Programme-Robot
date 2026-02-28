set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Définition du préfixe de compilation croisée
set(CROSS_COMPILE_PREFIX aarch64-linux-gnu)

# Spécification des compilateurs
set(CMAKE_C_COMPILER ${CROSS_COMPILE_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE_PREFIX}-g++)

# Où chercher les librairies cibles (sysroot)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Ajout d'une définition pour ton code (remplace -D__CROSS_COMPILE_ARM__)
add_definitions(-D__CROSS_COMPILE_ARM__)
