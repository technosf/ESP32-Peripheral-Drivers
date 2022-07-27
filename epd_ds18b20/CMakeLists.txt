idf_component_register(SRCS 
							"DS18B20.cpp" 
                    INCLUDE_DIRS 
                    		"."
                    )
                    
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-int-in-bool-context)

# List of components needed for the error codes list
set(optional_reqs epd_onewire)

idf_build_get_property(build_components BUILD_COMPONENTS)
foreach(req ${optional_reqs})
    if(req IN_LIST build_components)
        idf_component_get_property(req_lib ${req} COMPONENT_LIB)
        target_link_libraries(${COMPONENT_LIB} PRIVATE ${req_lib})
    endif()
endforeach()
