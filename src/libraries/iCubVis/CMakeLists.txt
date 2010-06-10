project(iCubVis)

file(GLOB_RECURSE folder_source *.c *.cc *.cpp)

file(GLOB folder_public_headers 
    blobDescriptor/include/iCub/vis/*.h
    camCalib/include/iCub/vis/*.h
    colorSegmentation/include/iCub/vis/*.h
    cvBlobsLib/include/iCub/vis/*.h
    faceEyeDetector/include/iCub/vis/*.h
    faceTracker/include/iCub/vis/*.h
    fastFilt/include/iCub/vis/*.h
    fourierVision/include/iCub/vis/*.h
    optFlow/include/iCub/vis/*.h
    salience/include/iCub/vis/*.h
    spMap/include/iCub/vis/*.h
)

file(GLOB folder_private_headers 
    blobDescriptor/include/private/*.h
    camCalib/include/private/*.h
    colorSegmentation/include/private/*.h
    cvBlobsLib/include/private/*.h
    faceEyeDetector/include/private/*.h
    faceTracker/include/private/*.h
    fastFilt/include/private/*.h
    fourierVision/include/private/*.h
    optFlow/include/private/*.h
    salience/include/private/*.h
    spMap/include/private/*.h
)

set(folder_header ${folder_public_headers} ${folder_private_headers})

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

set(iCubVis_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/blobDescriptor/include
    ${PROJECT_SOURCE_DIR}/camCalib/include
    ${PROJECT_SOURCE_DIR}/colorSegmentation/include
    ${PROJECT_SOURCE_DIR}/cvBlobsLib/include
    ${PROJECT_SOURCE_DIR}/faceEyeDetector/include
    ${PROJECT_SOURCE_DIR}/faceTracker/include
    ${PROJECT_SOURCE_DIR}/fastFilt/include
    ${PROJECT_SOURCE_DIR}/fourierVision/include
    ${PROJECT_SOURCE_DIR}/optFlow/include
    ${PROJECT_SOURCE_DIR}/salience/include
    ${PROJECT_SOURCE_DIR}/spMap/include
    CACHE STRING "iCubVis include directories"
)

include_directories(
    ${PROJECT_SOURCE_DIR}/blobDescriptor/include
    ${PROJECT_SOURCE_DIR}/camCalib/include
    ${PROJECT_SOURCE_DIR}/colorSegmentation/include
    ${PROJECT_SOURCE_DIR}/cvBlobsLib/include
    ${PROJECT_SOURCE_DIR}/faceEyeDetector/include
    ${PROJECT_SOURCE_DIR}/faceTracker/include
    ${PROJECT_SOURCE_DIR}/fastFilt/include
    ${PROJECT_SOURCE_DIR}/fourierVision/include
    ${PROJECT_SOURCE_DIR}/optFlow/include
    ${PROJECT_SOURCE_DIR}/salience/include
    ${PROJECT_SOURCE_DIR}/spMap/include
)

include_directories(
    ${PROJECT_SOURCE_DIR}/blobDescriptor/include/private
    ${PROJECT_SOURCE_DIR}/camCalib/include/private
    ${PROJECT_SOURCE_DIR}/colorSegmentation/include/private
    ${PROJECT_SOURCE_DIR}/cvBlobsLib/include/private
    ${PROJECT_SOURCE_DIR}/faceEyeDetector/include/private
    ${PROJECT_SOURCE_DIR}/faceTracker/include/private
    ${PROJECT_SOURCE_DIR}/fastFilt/include/private
    ${PROJECT_SOURCE_DIR}/fourierVision/include/private
    ${PROJECT_SOURCE_DIR}/optFlow/include/private
    ${PROJECT_SOURCE_DIR}/salience/include/private
    ${PROJECT_SOURCE_DIR}/spMap/include/private
)

include_directories(${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(iCubVis ${folder_source} ${folder_header})

target_link_libraries(iCubVis ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

icub_export_library(iCubVis 
                    INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include 
                    EXTERNAL_INCLUDE_DIRS ${iCubVis_INCLUDE_DIRS}
                    DESTINATION include/iCub/iCubVis
                    VERBOSE 
                    FILES ${folder_public_headers}
)

install(TARGETS iCubVis DESTINATION lib)
