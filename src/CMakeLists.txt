###########
# SOURCES #
###########

set(LIB_SRCS
        planning/Planner.cpp
        planning/ValidityChecker.cpp
        planning/SailboatStatePropagator.cpp
        util/AngleCalc.cpp
        )

set(LIB_HDRS
        planning/Planner.h
        planning/Obstacle.h
        planning/ValidityChecker.h
        planning/SailboatStatePropagator.H
        util/AngleCalc.h)



# Define the src_core library target
add_library(src_core STATIC ${LIB_SRCS} ${LIB_HDRS})

# Link dependencies
add_dependencies(src_core protofiles)

target_link_libraries(src_core ${CORE_LIBS})
