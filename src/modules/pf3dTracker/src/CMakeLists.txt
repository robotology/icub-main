
add_executable(eye2RootFrameTransformer eye2RootFrameTransformer.cpp)
target_link_libraries(eye2RootFrameTransformer iKin ${YARP_LIBRARIES})

add_executable(pf3dTracker pf3dTrackerMain.cpp pf3dTrackerSupport.cpp pf3dTracker.cpp)
target_link_libraries(pf3dTracker ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

#add_executable(trackerExpressions trackerExpressionsMain.cpp trackerExpressions.cpp)
#target_link_libraries(trackerExpressions ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS eye2RootFrameTransformer DESTINATION bin)
install(TARGETS pf3dTracker              DESTINATION bin)
#install(TARGETS trackerExpressions       DESTINATION bin)

