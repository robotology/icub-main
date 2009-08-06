add_library(pf3dTrackerSupportLib pf3dTrackerSupport.cpp)
add_library(leftEyeToRootLib leftEyeToRoot.cpp)
add_library(pf3dTrackerLib pf3dTracker.cpp)
add_library(trackerExpressionsLib trackerExpressions.cpp)

add_executable(leftEyeToRoot leftEyeToRootMain.cpp)
target_link_libraries(leftEyeToRoot leftEyeToRootLib ${OPENCV_LIBRARIES} ${IKIN_LIBRARIES})

add_executable(pf3dTracker pf3dTrackerMain.cpp)
target_link_libraries(pf3dTracker pf3dTrackerLib pf3dTrackerSupportLib ${OPENCV_LIBRARIES} ${IKIN_LIBRARIES})

add_executable(trackerExpressions trackerExpressionsMain.cpp)
target_link_libraries(trackerExpressions trackerExpressionsLib ${OPENCV_LIBRARIES} ${IKIN_LIBRARIES})

install(TARGETS leftEyeToRoot DESTINATION bin)
install(TARGETS pf3dTracker DESTINATION bin)
install(TARGETS trackerExpressions DESTINATION bin)

