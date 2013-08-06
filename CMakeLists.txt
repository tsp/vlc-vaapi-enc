cmake_minimum_required(VERSION 2.8)
project(vlc-vaapi-enc)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})

set(vlc-vaapi-enc_SOURCES test.cpp)

find_package(Libva REQUIRED X11 DRM)
find_package(VlcPlugin REQUIRED)

add_definitions(${Libva_DEFINITIONS} ${VlcPlugin_DEFINITIONS})
include_directories(${Libva_INCLUDE_DIRS} ${VlcPlugin_INCLUDE_DIRS})

add_executable(vlc-vaapi-enc ${vlc-vaapi-enc_SOURCES})
target_link_libraries(vlc-vaapi-enc ${Libva_LIBRARIES} ${VlcPlugin_LIBRARIES})

