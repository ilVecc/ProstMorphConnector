project(prost_redis_cpp)

cmake_minimum_required(VERSION 3.4)

# <---------- set c++ standard ------------->
# NOTE: you must build redis-plus-plus and your application code with the same standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

message(STATUS "project name: " ${PROJECT_NAME})
message(STATUS "project bin dir: " ${PROJECT_BINARY_DIR})
message(STATUS "project src dir: " ${PROJECT_SOURCE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/src)

# executable
file(GLOB SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)
target_link_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)


# https://github.com/sewenew/redis-plus-plus#build-with-cmake

# <------------ hiredis  --------------->
find_path(HIREDIS_HEADER hiredis)
target_include_directories(${PROJECT_NAME} PUBLIC ${HIREDIS_HEADER})
find_library(HIREDIS_LIB hiredis)
target_link_libraries(${PROJECT_NAME} PUBLIC ${HIREDIS_LIB})

# <------------ redis-plus-plus -------------->
# NOTE: this should be *sw* NOT *redis++*
find_path(REDIS_PLUS_PLUS_HEADER sw)
target_include_directories(${PROJECT_NAME} PUBLIC ${REDIS_PLUS_PLUS_HEADER})
find_library(REDIS_PLUS_PLUS_LIB redis++)
target_link_libraries(${PROJECT_NAME} PUBLIC ${REDIS_PLUS_PLUS_LIB})

# <------------ opencv -------------->
find_package(OpenCV REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_HEADER})
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

# <------------ itk -------------->
find_package(ITK REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC ${ITK_HEADER})
target_link_libraries(${PROJECT_NAME} PUBLIC ${ITK_LIBRARIES})
