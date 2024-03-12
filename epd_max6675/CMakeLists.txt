idf_component_register(
  SRCS "MAX6675.cpp"
  INCLUDE_DIRS "."
  PRIV_REQUIRES "driver"
)
                    
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-int-in-bool-context)

