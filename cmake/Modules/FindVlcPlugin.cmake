# - Try to find libvlccore for vlc plugins
# Once done, this will define
#
#  VlcPlugin_FOUND - System has libvlccore
#  VlcPlugin_INCLUDE_DIRS - The VlcPlugin include dirs
#  VlcPlugin_LIBRARIES - The libs needed for VlcPlugins
#  VlcPlugin_DEFINITIONS - The compiler switches needed for VlcPlugins

include(LibFindMacros)

if(NOT VlcPlugin_MODULE_STRING)
    message(FATAL_ERROR "You need to set VlcPlugin_MODULE_STRING to build a vlc plugin!")
endif()

libfind_pkg_check_modules(VlcPlugin_PKGCONF vlc-plugin)

set(VlcPlugin_DEFINITIONS ${VlcPlugin_PKGCONF_CFLAGS_OTHER} -DMODULE_STRING=\"${VlcPlugin_MODULE_STRING}\")
find_path(VlcPlugin_INCLUDE_DIR NAMES vlc_plugin.h PATHS ${VlcPlugin_PKGCONF_INCLUDE_DIRS})
find_library(VlcPlugin_LIBRARY NAMES vlccore PATHS ${VlcPlugin_PKGCONF_LIBRARY_DIRS})

set(VlcPlugin_PROCESS_INCLUDES VlcPlugin_INCLUDE_DIR)
set(VlcPlugin_PROCESS_LIBS VlcPlugin_LIBRARY)
libfind_process(VlcPlugin)

