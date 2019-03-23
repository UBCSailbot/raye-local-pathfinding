
set(TEST_FILES
        planning/PlannerTest.cpp)

find_package(rostest REQUIRED)

catkin_add_gtest(run_basic_tests ${TEST_FILES})

target_link_libraries(run_basic_tests src_core gtest_main ${catkin_LIBRARIES})
