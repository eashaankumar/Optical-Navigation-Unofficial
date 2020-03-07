#However, the file(GLOB...) allows for wildcard additions:
#file(GLOB SOURCES "*.cpp")

set(SOURCES test_controller.cpp controller.cpp ../acquisition/acquisition.cpp ../cam_meas/cam_meas.cpp ../find/find.cpp ../ukf/ukf.cpp)
 
add_executable(${CONTROLLER_BINARY}_run ${SOURCES})
set_target_properties(${CONTROLLER_BINARY}_run PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

target_link_libraries(${CONTROLLER_BINARY}_run ${OpenCV_LIBS})

ADD_CUSTOM_COMMAND(
    OUTPUT "${CONTROLLER_BINARY}_run"
    COMMAND ${CONTROLLER_BINARY}_run
)