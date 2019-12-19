


set(COMPONENT_SRCS "firmware_main.c" "bsec_serialized_configurations_iaq.c" "firmware_connection.c" "bsec_integration.c" "bme680.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")


#set(ULP_APP_NAME ulp_${COMPONENT_NAME})
#set(ULP_S_SOURCES ulp/ulp_assembly_source_file.S)
#set(ULP_EXP_DEP_SRCS "ulp_c_source_file.c")
#include(${IDF_PATH}/components/ulp/component_ulp_common.cmake)

register_component()

# edit this according to your lib path
target_link_libraries(${COMPONENT_TARGET} INTERFACE "/home/ignne/stuff/iot/firmware-sdk/main/libalgobsec.a")
target_link_libraries(${COMPONENT_TARGET} INTERFACE "-lm" "-lgcc")
