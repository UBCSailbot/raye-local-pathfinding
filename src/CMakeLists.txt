###########
# SOURCES #
###########

set(LIB_SRCS
        example/Example.cpp
        example/Planner.cpp )

set(LIB_HDRS
        example/Example.h
        example/Planner.h
        )

# Define the src_core library target
add_library(src_core STATIC ${LIB_SRCS} ${LIB_HDRS})

# Link dependencies
add_dependencies(src_core protofiles)

target_link_libraries(src_core ${CORE_LIBS})
