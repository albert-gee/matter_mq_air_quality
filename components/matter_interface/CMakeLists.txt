idf_component_register(
    SRCS "src/matter_manager.cpp" "src/matter_air_quality.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_matter
)

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
