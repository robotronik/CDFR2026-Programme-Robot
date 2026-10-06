# Target Device Configuration (customizable via -DPI_HOST=... or cache variables)
set(PI_USER "robotronik" CACHE STRING "Target device SSH username")
set(PI_HOST "172.27.123.146" CACHE STRING "Target device IP or hostname")
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
        COMMAND ${SSH_CMD} -t ${PI_USER}@${PI_HOST}
                "cd ${PI_DIR}/${PI_DEST} && chmod +x autoRunInstaller.sh && \
                 sudo ./autoRunInstaller.sh --uninstall ${PI_DIR}/${PI_DEST}/programCDFR && \
                 sudo ./autoRunInstaller.sh --install ${PI_DIR}/${PI_DEST}/programCDFR"
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
