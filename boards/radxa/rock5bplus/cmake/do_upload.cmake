function(get_autopilot_credentials)
    if(DEFINED ENV{AUTOPILOT_HOST})
        set(AUTOPILOT_HOST $ENV{AUTOPILOT_HOST} PARENT_SCOPE)
    else()
        message(STATUS "have no env AUTOPILOT_HOST, please input host:")
        execute_process(
            COMMAND bash -c "read -r ip && echo \$ip"
            OUTPUT_VARIABLE AUTOPILOT_HOST_INPUT
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(AUTOPILOT_HOST_INPUT STREQUAL "")
            message(FATAL_ERROR "invalid host, abort")
        endif()
        set(AUTOPILOT_HOST ${AUTOPILOT_HOST_INPUT} PARENT_SCOPE)
    endif()

    if(DEFINED ENV{AUTOPILOT_USER})
        set(AUTOPILOT_USER $ENV{AUTOPILOT_USER} PARENT_SCOPE)
    else()
        set(AUTOPILOT_USER "radxa" PARENT_SCOPE)
    endif()
endfunction()

get_autopilot_credentials()

execute_process(
    COMMAND ssh ${AUTOPILOT_USER}@${AUTOPILOT_HOST} "mkdir -p /home/${AUTOPILOT_USER}/px4"
    RESULT_VARIABLE mkdir_result
)
if(NOT mkdir_result EQUAL 0)
    message(FATAL_ERROR "upload failed")
endif()

execute_process(
    COMMAND rsync -arh --progress ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ${PX4_SOURCE_DIR}/posix-configs/radxa/rock5bplus.config ${PX4_BINARY_DIR}/etc "${AUTOPILOT_USER}@${AUTOPILOT_HOST}:/home/${AUTOPILOT_USER}/px4"
    RESULT_VARIABLE scp_result
)
if(NOT scp_result EQUAL 0)
    message(FATAL_ERROR "upload failed")
endif()
