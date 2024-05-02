project(CodeCraftSDK)
cmake_minimum_required (VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)

find_package(CURL REQUIRED)
include_directories(${CURL_INCLUDE_DIRS})

AUX_SOURCE_DIRECTORY(. src)
ADD_EXECUTABLE(main ${src})

target_link_libraries(main ${CURL_LIBRARIES})