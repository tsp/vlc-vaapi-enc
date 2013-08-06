cmake_minimum_required(VERSION 2.8)
project(vlc-vaapi-enc)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})

set(vlc-vaapi-enc_SOURCES
    src/plugin.cpp
    src/encoder.cpp
    src/copy.c)
set(vlc-vaapi-enc_HEADERS
    src/encoder.h
    src/copy.h)

find_package(Libva REQUIRED X11 DRM)
find_package(VlcPlugin REQUIRED)
find_package(CXX11 REQUIRED)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${CXX11_FLAGS})
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} -std=gnu99)
add_definitions(${Libva_DEFINITIONS} ${VlcPlugin_DEFINITIONS})
include_directories(${Libva_INCLUDE_DIRS} ${VlcPlugin_INCLUDE_DIRS} src)

add_definitions(-DMODULE_STRING=\"h264-vaapi-enc\")

add_library(vlc-vaapi-enc ${vlc-vaapi-enc_SOURCES} ${vlc-vaapi-enc_HEADERS})
target_link_libraries(vlc-vaapi-enc ${Libva_LIBRARIES} ${VlcPlugin_LIBRARIES})

install_targets(/usr/lib/vlc/plugins/codec vlc-vaapi-enc)
