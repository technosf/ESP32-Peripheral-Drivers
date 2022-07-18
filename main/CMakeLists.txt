set(srcs "app_main.cpp"
							#"DHT22.cpp" 
							#"DS18B20.cpp" 
    "MAX6675.cpp"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "include")
                    
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-int-in-bool-context)
