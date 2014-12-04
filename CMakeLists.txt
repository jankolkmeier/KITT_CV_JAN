cmake_minimum_required(VERSION 2.8)
project(KITT_CV_JAN)

set(KCVJ_GUI 1)

file(GLOB SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB HEADERS ${PROJECT_SOURCE_DIR}/headers/*.h)     

configure_file (
    ${PROJECT_SOURCE_DIR}/Config.h.in
    ${PROJECT_SOURCE_DIR}/headers/Config.h
)

include_directories(${PROJECT_SOURCE_DIR}/src)      
include_directories(${PROJECT_SOURCE_DIR}/headers)      

find_package(OpenCV REQUIRED)

add_executable(kcvj ${SRC} ${HEADERS})
target_link_libraries(kcvj ${OpenCV_LIBS})
