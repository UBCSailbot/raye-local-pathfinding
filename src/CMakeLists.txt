###########
# SOURCES #
###########

set(LIB_SRCS
        planning/Planner.cpp
        planning/ValidityChecker.cpp
        planning/SailboatStatePropagator.cpp
        util/AngleCalc.cpp
        planning/SailboatGoalRegion.cpp
        util/CoordinateTranslator.cpp
        util/UTM.cpp
        )

set(LIB_HDRS
        planning/Planner.h
        datatypes/Obstacle.h
        planning/ValidityChecker.h
        planning/SailboatStatePropagator.h
        util/AngleCalc.h
        planning/SailboatGoalRegion.h
        datatypes/Coordinate.h
        util/CoordinateTranslator.h
        util/UTM.h ../test/basic_tests/util/CoordinateTranslatorTest.cpp ../test/basic_tests/util/CoordinateTranslatorTest.h planning/TimeObjective.cpp planning/TimeObjective.h)



# Define the src_core library target
add_library(src_core STATIC ${LIB_SRCS} ${LIB_HDRS})

# Link dependencies
add_dependencies(src_core protofiles)

target_link_libraries(src_core ${CORE_LIBS})
