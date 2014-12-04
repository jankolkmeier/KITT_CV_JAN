cmake_minimum_required(VERSION 2.8)
project(KITT_CV_JAN)

file(GLOB SRC_LIB ${PROJECT_SOURCE_DIR}/kcvj/src/*.cpp)
file(GLOB HEADERS_LIB ${PROJECT_SOURCE_DIR}/kcvj/headers/*.h)     
file(GLOB SRC_DEMO ${PROJECT_SOURCE_DIR}/kcvj_demo/src/*.cpp)
file(GLOB HEADERS_DEMO ${PROJECT_SOURCE_DIR}/kcvj_demo/headers/*.h)     
file(GLOB SRC_MIN ${PROJECT_SOURCE_DIR}/kcvj_minimal/src/*.cpp)
file(GLOB HEADERS_MIN ${PROJECT_SOURCE_DIR}/kcvj_minimal/headers/*.h)     

include_directories(${PROJECT_SOURCE_DIR}/kcvj/src)      
include_directories(${PROJECT_SOURCE_DIR}/kcvj/headers)      
include_directories(${PROJECT_SOURCE_DIR}/kcvj_demo/src)      
include_directories(${PROJECT_SOURCE_DIR}/kcvj_demo/headers)      
include_directories(${PROJECT_SOURCE_DIR}/kcvj_min/src)      
include_directories(${PROJECT_SOURCE_DIR}/kcvj_min/headers)      

find_package(OpenCV REQUIRED)

add_library(kcvj ${SRC_LIB} ${HEADERS_LIB})

add_executable(kcvj_demo ${SRC_DEMO} ${HEADERS_DEMO})
target_link_libraries(kcvj_demo ${OpenCV_LIBS} kcvj)

add_executable(kcvj_minimal ${SRC_MIN} ${HEADERS_MIN})
target_link_libraries(kcvj_minimal ${OpenCV_LIBS} kcvj)
