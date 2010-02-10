
add_library(pf3dTrackerSupportLib pf3dTrackerSupport.cpp)
add_library(pf3dTrackerLib pf3dTracker.cpp)
add_library(trackerExpressionsLib trackerExpressions.cpp)

add_executable(eye2RootFrameTransformer eye2RootFrameTransformer.cpp)
target_link_libraries(eye2RootFrameTransformer ${IKIN_LIBRARIES})

add_executable(pf3dTracker pf3dTrackerMain.cpp)
target_link_libraries(pf3dTracker pf3dTrackerLib pf3dTrackerSupportLib ${OPENCV_LIBRARIES})

add_executable(trackerExpressions trackerExpressionsMain.cpp)
target_link_libraries(trackerExpressions trackerExpressionsLib ${OPENCV_LIBRARIES})

install(TARGETS pf3dTrackerSupportLib    DESTINATION lib)
install(TARGETS pf3dTrackerLib           DESTINATION lib)
install(TARGETS trackerExpressionsLib    DESTINATION lib)

install(TARGETS eye2RootFrameTransformer DESTINATION bin)
install(TARGETS pf3dTracker              DESTINATION bin)
install(TARGETS trackerExpressions       DESTINATION bin)

