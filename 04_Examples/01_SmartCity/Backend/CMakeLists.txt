cmake_minimum_required(VERSION 2.8)
project( MiniBots )

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_REQUIRED_FLAGS -std=c++17)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(THREADS_PREFER_PTHREAD_FLAG ON)

add_executable( minibots src/map.cc src/tracker.cc src/minibots.cc src/server.cc)
add_executable( trackBuilder src/trackBuilder.cc )
set_property(TARGET minibots PROPERTY CXX_STANDARD 17)
set_property(TARGET trackBuilder PROPERTY CXX_STANDARD 17)

find_package(Threads)
find_package(OpenCV REQUIRED)
find_package(SDL REQUIRED sdl)
find_package(SDL_image REQUIRED SDL_image>=1.2.0)
find_package(SDL_net REQUIRED SDL_net>=1.2.0)

find_path(APRILTAG_INCLUDE_DIR NAMES apriltag/apriltag.h)
find_library(APRILTAG_LIBRARY NAMES apriltag)

find_path(YAMLCPP_INCLUDE_DIR NAMES yaml-cpp/yaml.h)
find_library(YAMLCPP_LIBRARY NAMES yaml-cpp)

find_path(LIBCONFIGCPP_INCLUDE_DIR NAMES libconfig.h++)
find_library(LIBCONFIGCPP_LIBRARY NAMES config++)

find_path(LIBPOWERGINE_INCLUDE_DIR NAMES powergine/powergine.h)
find_library(LIBPOWERGINE_LIBRARY NAMES powergine)

target_include_directories(
    minibots PRIVATE 
    ${OpenCV_INCLUDE_DIRS} 
    ${YAMLCPP_INCLUDE_DIR}
    ${APRILTAG_INCLUDE_DIR}
    ${OPENSSL_INCLUDE_DIR}
    ${LIBCONFIGCPP_INCLUDE_DIR}
    ${LIBPOWERGINE_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)
target_include_directories(
    trackBuilder PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIR}
    ${APRILTAG_INCLUDE_DIR}
    ${LIBCONFIGCPP_INCLUDE_DIR}
    ${OPENSSL_INCLUDE_DIR}
    ${LIBPOWERGINE_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
) 

target_link_libraries( 
    minibots 
    ${OpenCV_LIBS}
    ${APRILTAG_LIBRARY}
    ${YAMLCPP_LIBRARY}
    ${LIBCONFIGCPP_LIBRARY}
    ${OPENSSL_LIBRARY}
    ${LIBPOWERGINE_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${SDL_LIBRARY}
    ${SDL_NET_LIBRARIES}
    ${SDL_IMAGE_LIBRARIES}
)
target_link_libraries( 
    trackBuilder 
    ${OpenCV_LIBS}
    ${APRILTAG_LIBRARY}
    ${YAMLCPP_LIBRARY}
    ${LIBCONFIGCPP_LIBRARY}
    ${OPENSSL_LIBRARY}
    ${LIBPOWERGINE_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${SDL_LIBRARY}
    ${SDL_NET_LIBRARIES}
    ${SDL_IMAGE_LIBRARIES}
)
