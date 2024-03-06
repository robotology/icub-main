project(MotorTemperaturePublisher)
# find YARP
find_package(YARP COMPONENTS os sig dev)
# set up our program
add_executable(${PROJECT_NAME} ./src/MotorTemperaturePublisher.h ./src/MotorTemperaturePublisher.cpp  ./src/main.cpp )

# link with YARP libraries
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
