idf_component_register(SRCS 
							"OneWireDevice.cpp" 
							"OneWireBus.cpp" 
							"OneWireBusRMT.cpp" 
                    INCLUDE_DIRS 
                    		"include"
                    )
                    
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-int-in-bool-context)
