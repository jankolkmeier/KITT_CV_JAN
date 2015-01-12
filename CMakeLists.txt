cmake_minimum_required(VERSION 2.8)
project(KITT_CV_JAN)

file(GLOB SRC_LIB ${PROJECT_SOURCE_DIR}/kcvj/src/*.cpp)
file(GLOB HEADERS_LIB ${PROJECT_SOURCE_DIR}/kcvj/headers/*.h)     
file(GLOB SRC_DEMO ${PROJECT_SOURCE_DIR}/kcvj_demo/src/*.cpp)
file(GLOB HEADERS_DEMO ${PROJECT_SOURCE_DIR}/kcvj_demo/headers/*.h)     
file(GLOB SRC_MIN ${PROJECT_SOURCE_DIR}/kcvj_minimal/src/*.cpp)
file(GLOB HEADERS_MIN ${PROJECT_SOURCE_DIR}/kcvj_minimal/headers/*.h)     

file(GLOB SRC_STREAM ${PROJECT_SOURCE_DIR}/kcvj_stream/src/*.cpp)
file(GLOB HEADERS_STREAM ${PROJECT_SOURCE_DIR}/kcvj_stream/headers/*.h)

file(GLOB SRC_MC ${PROJECT_SOURCE_DIR}/kcvj_multicam/src/*.cpp)
file(GLOB HEADERS_MC ${PROJECT_SOURCE_DIR}/kcvj_multicam/headers/*.h)     

file(GLOB SRC_CALIB ${PROJECT_SOURCE_DIR}/calibration/src/*.cpp)
file(GLOB HEADERS_CALIB ${PROJECT_SOURCE_DIR}/calibration/headers/*.h)     

file(GLOB SRC_ARUCO ${PROJECT_SOURCE_DIR}/aruco_test/src/*.cpp)
file(GLOB HEADERS_ARUCO ${PROJECT_SOURCE_DIR}/aruco_test/headers/*.h)     

file(GLOB SRC_FEATURE ${PROJECT_SOURCE_DIR}/feature_test/src/*.cpp)
file(GLOB HEADERS_FEATURE ${PROJECT_SOURCE_DIR}/feature_test/headers/*.h)     

file(GLOB SRC_HOUGH ${PROJECT_SOURCE_DIR}/hough_circle_test/src/*.cpp)
file(GLOB HEADERS_HOUGH ${PROJECT_SOURCE_DIR}/hough_circle_test/headers/*.h)     



include_directories(${PROJECT_SOURCE_DIR}/kcvj/headers)      
include_directories(${PROJECT_SOURCE_DIR}/kcvj_demo/headers)      
include_directories(${PROJECT_SOURCE_DIR}/kcvj_minimal/headers)      
include_directories(${PROJECT_SOURCE_DIR}/kcvj_multicam/headers)      

include_directories(${PROJECT_SOURCE_DIR}/calibration/headers)      
include_directories(${PROJECT_SOURCE_DIR}/aruco_test/headers)      
include_directories(${PROJECT_SOURCE_DIR}/feature_test/headers)      
include_directories(${PROJECT_SOURCE_DIR}/hough_circle_test/headers)      

if(WIN32)  
include_directories(${PROJECT_SOURCE_DIR}/kcvj_stream/headers) 
endif()

find_package(OpenCV REQUIRED)
add_library(kcvj ${SRC_LIB} ${HEADERS_LIB})

add_executable(kcvj_stream ${SRC_STREAM} ${HEADERS_STREAM})
add_executable(kcvj_demo ${SRC_DEMO} ${HEADERS_DEMO})

if(WIN32)
  target_link_libraries(kcvj_demo ${OpenCV_LIBS} kcvj ws2_32)
  target_link_libraries(kcvj_stream ${OpenCV_LIBS} kcvj  ws2_32)
else()
  find_package(Threads)
  target_link_libraries(kcvj_demo ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} kcvj)
  target_link_libraries(kcvj_stream ${OpenCV_LIBS} kcvj)
endif()

add_executable(kcvj_minimal ${SRC_MIN} ${HEADERS_MIN})
target_link_libraries(kcvj_minimal ${OpenCV_LIBS} kcvj)

add_executable(kcvj_multicam ${SRC_MC} ${HEADERS_MC})
target_link_libraries(kcvj_multicam ${OpenCV_LIBS} kcvj)

add_executable(calibration ${SRC_CALIB} ${HEADERS_CALIB})
target_link_libraries(calibration ${OpenCV_LIBS})

add_executable(aruco_test ${SRC_ARUCO} ${HEADERS_ARUCO})
target_link_libraries(aruco_test ${OpenCV_LIBS} kcvj)

add_executable(feature_test ${SRC_FEATURE} ${HEADERS_FEATURE})
target_link_libraries(feature_test ${OpenCV_LIBS} kcvj)

add_executable(hough_circle_test ${SRC_HOUGH} ${HEADERS_HOUGH})
target_link_libraries(hough_circle_test ${OpenCV_LIBS} kcvj)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread")

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -mfpu=neon")

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -g")
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -g")

#set(CMAKE_BUILD_TYPE Debug)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
