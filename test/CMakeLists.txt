set(TEST_SOURCES ../src/find/find.cpp ../src/ukf/ukf.cpp)
add_library(${CMAKE_PROJECT_NAME}_lib STATIC ${TEST_SOURCES})

set(SOURCES 
    find_tests.cpp
    ukf_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/ukf/ukf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/find/find.cpp
)

add_executable(${TEST_BINARY}_run ${SOURCES})
set_target_properties(${TEST_BINARY}_run PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

target_link_libraries( ${TEST_BINARY}_run PUBLIC
    ${OpenCV_LIBS} 
    Eigen3::Eigen
    gtest_main
)

add_test(
    NAME ${TEST_BINARY}_run
    COMMAND ${TEST_BINARY}_run
)


