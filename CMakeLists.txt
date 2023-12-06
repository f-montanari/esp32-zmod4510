set(srcs "ZMOD4510.c"
		 "src/esp32_i2c.c"
		 "src/zmod4xxx.c"
	)

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES driver)

# Link library file to component
add_library(lib_oaq_2nd_gen STATIC IMPORTED)
set_property(TARGET lib_oaq_2nd_gen PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_oaq_2nd_gen.a")
target_link_libraries(${COMPONENT_TARGET} PUBLIC lib_oaq_2nd_gen)