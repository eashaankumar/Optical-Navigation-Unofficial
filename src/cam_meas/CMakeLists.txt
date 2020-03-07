#set(LIB_SOURCES ../find/find.cpp)
#add_library(${CMAKE_PROJECT_NAME}_cammeas_lib STATIC ${LIB_SOURCES})

#Can manually add the sources using the set command as follows:
set(SOURCES test_cammeas.cpp cam_meas.cpp ../find/find.cpp)

#However, the file(GLOB...) allows for wildcard additions:
#file(GLOB SOURCES "*.cpp")
 
add_executable(${CAMMEAS_BINARY}_run ${SOURCES})
set_target_properties(${CAMMEAS_BINARY}_run PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

target_link_libraries(${CAMMEAS_BINARY}_run ${OpenCV_LIBS})

ADD_CUSTOM_COMMAND(
    OUTPUT "${CAMMEAS_BINARY}_run"
    COMMAND ${CAMMEAS_BINARY}_run
)