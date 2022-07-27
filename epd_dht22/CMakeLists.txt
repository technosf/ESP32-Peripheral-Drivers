idf_component_register(SRCS 
							"DHT22.cpp" 
                    INCLUDE_DIRS 
                    		"."
                    )
                    
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-int-in-bool-context)

