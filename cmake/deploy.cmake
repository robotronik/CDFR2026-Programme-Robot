# Target Device Configuration (customizable via -DPI_HOST=... or cache variables)
set(PI_USER "robotronik" CACHE STRING "Target device SSH username")
set(PI_HOST "192.168.8.108" CACHE STRING "Target device IP or hostname")
set(PI_DIR  "/home/${PI_USER}/CDFR" CACHE STRING "Target installation base directory")
set(PI_DEST "arm_bin" CACHE STRING "Target installation subdirectory")

find_program(SSH_CMD ssh)
find_program(RSYNC_CMD rsync)

if(TARGET programCDFR AND SSH_CMD AND RSYNC_CMD)
    # Each ssh COMMAND below is a single argument with no shell metacharacters:
    # CMake joins COMMANDs with '&&' and runs the rule through the local shell,
    # so a remote string such as "cd dir && ./script" would be split and its
    # parts executed locally. Sequencing is therefore expressed as separate
    # COMMANDs, and the installer is invoked via an absolute path (rsync -a
    # preserves its executable bit) so no remote chmod is needed.
    add_custom_target(deploy
        COMMAND ${SSH_CMD} ${PI_USER}@${PI_HOST} "mkdir -p ${PI_DIR}/${PI_DEST}"
        COMMAND ${RSYNC_CMD} -az --progress --delete
                ${CMAKE_BINARY_DIR}/programCDFR
                ${CMAKE_BINARY_DIR}/html
                ${CMAKE_BINARY_DIR}/data
                ${CMAKE_SOURCE_DIR}/autoRunInstaller.sh
                ${PI_USER}@${PI_HOST}:${PI_DIR}/${PI_DEST}/
        COMMAND ${SSH_CMD} -t ${PI_USER}@${PI_HOST}
                "sudo ${PI_DIR}/${PI_DEST}/autoRunInstaller.sh --uninstall ${PI_DIR}/${PI_DEST}/programCDFR"
        COMMAND ${SSH_CMD} -t ${PI_USER}@${PI_HOST}
                "sudo ${PI_DIR}/${PI_DEST}/autoRunInstaller.sh --install ${PI_DIR}/${PI_DEST}/programCDFR"
        DEPENDS programCDFR
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        USES_TERMINAL
        COMMENT "Deploying and restarting programCDFR on ${PI_USER}@${PI_HOST}..."
    )
endif()
