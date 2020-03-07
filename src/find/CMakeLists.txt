#Can manually add the sources using the set command as follows:
#set(SOURCES test_find.cpp find.cpp)

#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "*.cpp")
 
add_executable(${FIND_BINARY}_run ${SOURCES})
set_target_properties(${FIND_BINARY}_run PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

target_link_libraries(${FIND_BINARY}_run ${OpenCV_LIBS})

ADD_CUSTOM_COMMAND(
    OUTPUT "${FIND_BINARY}_run"
    COMMAND ${FIND_BINARY}_run
)