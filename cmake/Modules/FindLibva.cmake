# - Try to find libva
# Once done, this will define
#
#  Libva_FOUND - System has Libva
#  Libva_INCLUDE_DIRS - The Libva include  dirs
#  Libva_LIBRARIES - The libs needed by Libva
#  Libva_DEFINITIONS - The compiler switches needed by Libva

include(LibFindMacros)

set(_known_components DRM GLX X11)

if(Libva_FIND_COMPONENTS)
    foreach(component ${Libva_FIND_COMPONENTS})
        string(TOUPPER ${component} _COMPONENT)

        list(FIND _known_components ${_COMPONENT} _component_known)
        if(_component_known EQUAL -1)
            message(FATAL_ERROR "${_COMPONENT} is not a known libva module")
        endif()

        set(LIBVA_USE_${_COMPONENT} true)
    endforeach()
else()
    set(LIBVA_USE_X11 true)
endif()

if(LIBVA_USE_X11)
    libfind_package(Libva X11)
endif()

libfind_pkg_check_modules(Libva_PKGCONF libva)
find_path(Libva_INCLUDE_DIR NAMES va/va.h PATHS ${Libva_PKGCONF_INCLUDE_DIRS})
find_library(Libva_LIBRARY NAMES va PATHS ${Libva_PKGCONF_LIBRARY_DIRS})

set(Libva_PROCESS_INCLUDES Libva_INCLUDE_DIR)
set(Libva_PROCESS_LIBS Libva_LIBRARY)

if(LIBVA_USE_X11)
    libfind_pkg_check_modules(Libva_X11_PKGCONF libva-x11)

    find_path(Libva_X11_INCLUDE_DIR NAMES va/va_x11.h PATHS ${Libva_X11_PKGCONF_INCLUDE_DIRS})
    find_library(Libva_X11_LIBRARY NAMES va-x11 PATHS ${Libva_X11_PKGCONF_LIBRARY_DIRS})

    set(Libva_PROCESS_INCLUDES ${Libva_PROCESS_INCLUDES} Libva_X11_INCLUDE_DIR X11_INCLUDE_DIR)
    set(Libva_PROCESS_LIBS ${Libva_PROCESS_LIBS} Libva_X11_LIBRARY X11_LIBRARIES)
endif()

if(LIBVA_USE_GLX)
    libfind_pkg_check_modules(Libva_GLX_PKGCONF libva-glx)

    find_path(Libva_GLX_INCLUDE_DIR NAMES va/va_glx.h PATHS ${Libva_GLX_PKGCONF_INCLUDE_DIRS})
    find_library(Libva_GLX_LIBRARY NAMES va-glx PATHS ${Libva_GLX_PKGCONF_LIBRARY_DIRS})

    set(Libva_PROCESS_INCLUDES ${Libva_PROCESS_INCLUDES} Libva_GLX_INCLUDE_DIR)
    set(Libva_PROCESS_LIBS ${Libva_PROCESS_LIBS} Libva_GLX_LIBRARY)
endif()

if(LIBVA_USE_DRM)
    libfind_pkg_check_modules(Libva_DRM_PKGCONF libva-drm)

    find_path(Libva_DRM_INCLUDE_DIR NAMES va/va_drm.h PATHS ${Libva_DRM_PKGCONF_INCLUDE_DIRS})
    find_library(Libva_DRM_LIBRARY NAMES va-drm PATHS ${Libva_DRM_PKGCONF_LIBRARY_DIRS})

    set(Libva_PROCESS_INCLUDES ${Libva_PROCESS_INCLUDES} Libva_DRM_INCLUDE_DIR)
    set(Libva_PROCESS_LIBS ${Libva_PROCESS_LIBS} Libva_DRM_LIBRARY)
endif()

libfind_process(Libva)

