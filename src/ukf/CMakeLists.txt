#Can manually add the sources using the set command as follows:
set(SOURCES test_ukf.cpp ukf.cpp)
 
add_executable(${UKF_BINARY}_run ${SOURCES})
set_target_properties(${UKF_BINARY}_run PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

target_link_libraries( ${UKF_BINARY}_run ${OpenCV_LIBS} Eigen3::Eigen)

ADD_CUSTOM_COMMAND(
    OUTPUT "${UKF_BINARY}_run"
    COMMAND ${UKF_BINARY}_run
)