# Target Device Configuration (customizable via -DPI_HOST=... or cache variables)
set(PI_USER "robotronik" CACHE STRING "Target device SSH username")
set(PI_HOST "192.168.0.103" CACHE STRING "Target device IP or hostname")
set(PI_DIR  "/home/${PI_USER}/CDFR" CACHE STRING "Target installation base directory")
set(PI_DEST "arm_bin" CACHE STRING "Target installation subdirectory")

find_program(SSH_CMD ssh)
find_program(RSYNC_CMD rsync)

if(TARGET programCDFR AND SSH_CMD AND RSYNC_CMD)
    add_custom_target(deploy
        COMMAND ${SSH_CMD} ${PI_USER}@${PI_HOST} "mkdir -p ${PI_DIR}/${PI_DEST}"
        COMMAND ${RSYNC_CMD} -az --progress --delete
                ${CMAKE_BINARY_DIR}/programCDFR
                ${CMAKE_BINARY_DIR}/html
                ${CMAKE_BINARY_DIR}/data
                ${CMAKE_SOURCE_DIR}/autoRunInstaller.sh
                ${PI_USER}@${PI_HOST}:${PI_DIR}/${PI_DEST}/
        # Une seule commande par appel ssh : CMake échappe les espaces des
        # arguments mais pas les opérateurs shell (&&, ;), qui seraient alors
        # interprétés par le shell local au lieu du shell distant. On enchaîne
        # donc des appels séparés, avec des chemins absolus : autoRunInstaller.sh
        # retrouve son dossier via $0, un `cd` préalable n'est pas nécessaire.
        COMMAND ${SSH_CMD} ${PI_USER}@${PI_HOST}
                "chmod +x ${PI_DIR}/${PI_DEST}/autoRunInstaller.sh"
        COMMAND ${SSH_CMD} -t ${PI_USER}@${PI_HOST}
                "sudo ${PI_DIR}/${PI_DEST}/autoRunInstaller.sh --uninstall ${PI_DIR}/${PI_DEST}/programCDFR"
        COMMAND ${SSH_CMD} -t ${PI_USER}@${PI_HOST}
                "sudo ${PI_DIR}/${PI_DEST}/autoRunInstaller.sh --install ${PI_DIR}/${PI_DEST}/programCDFR"
        DEPENDS programCDFR
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        USES_TERMINAL
        COMMENT "Deploying and restarting programCDFR on ${PI_USER}@${PI_HOST}..."
    )

    add_custom_target(logs
        COMMAND ${SSH_CMD} -t ${PI_USER}@${PI_HOST} "journalctl -u programCDFR -f --output=cat"
        USES_TERMINAL
        COMMENT "Streaming robot journalctl logs from ${PI_USER}@${PI_HOST}..."
    )
endif()
