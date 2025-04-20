# CDDL HEADER START
#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#
# CDDL HEADER END

# Copyright 2024 Saso Kiselkov. All rights reserved.

cmake_minimum_required(VERSION 3.9)
project(openwxr)

option(LIBACFUTILS	"libacfutils repo")
option(OPENGPWS		"OpenGPWS repo")
option(FAST_DEBUG	"enable fast debug mode")

if(APPLE)
	set(PLAT_SHORT "mac64")
	set(PLAT_LONG "mac-64")
	set(PLAT_ONLY "mac")
	set(PLUGIN_BIN_OUTDIR "mac_x64")
elseif(WIN32)
	set(PLAT_SHORT "win64")
	set(PLAT_LONG "win-64")
	set(PLAT_ONLY "win")
	set(PLUGIN_BIN_OUTDIR "win_x64")
else()
	set(PLAT_SHORT "lin64")
	set(PLAT_LONG "linux-64")
	set(PLAT_ONLY "linux")
	set(PLUGIN_BIN_OUTDIR "lin_x64")
endif()

set(SRC
    atmo_xp11.c
    dbg_log.c
    fontmgr.c
    standalone.c
    wxr.c
    xplane.c
)
set(HDR
    atmo.h
    atmo_xp11.h
    dbg_log.h
    fontmgr.h
    standalone.h
    wxr.h
    xplane.h
)

set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} -DDEBUG -g")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG -O0 -g")

set(ALL_SRC ${SRC} ${HDR})
list(SORT ALL_SRC)

if(APPLE)
	add_executable(openwxr ${ALL_SRC})
else()
	add_library(openwxr SHARED ${ALL_SRC})
endif()

execute_process(COMMAND
    ${LIBACFUTILS}/pkg-config-deps ${PLAT_LONG} --cflags
    OUTPUT_VARIABLE DEP_CFLAGS)
string(REGEX REPLACE "\n$" "" DEP_CFLAGS "${DEP_CFLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${DEP_CFLAGS}")

execute_process(COMMAND
    ${LIBACFUTILS}/pkg-config-deps ${PLAT_LONG} --libs
    OUTPUT_VARIABLE DEP_LIBS)
string(REGEX REPLACE "\n$" "" DEP_LIBS "${DEP_LIBS}")

include_directories(openwxr PUBLIC
    "${LIBACFUTILS}/src"
    "../api"
    "${OPENGPWS}/api"
)

#compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror --std=c11 \
    -Wno-unused-local-typedefs -Wno-missing-field-initializers")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
    -DCHECK_RESULT_USED=\"__attribute__ ((warn_unused_result))\"")
add_definitions(-D_GNU_SOURCE -D_FILE_OFFSET_BITS=64)

if(${FAST_DEBUG})
	add_definitions(-DFAST_DEBUG=1)
else()
	add_definitions(-DFAST_DEBUG=0)
endif()

#libraries
include_directories(openwxr PUBLIC
    "${LIBACFUTILS}/SDK/CHeaders/XPLM"
    "${LIBACFUTILS}/SDK/CHeaders/Widgets"
    "${LIBACFUTILS}/SDK/CHeaders/Wrappers")

# Yes, we need to strip a trailing newline from command output.
execute_process(COMMAND git rev-parse --short HEAD
    OUTPUT_VARIABLE PLUGIN_VERSION)
string(REGEX REPLACE "\n$" "" PLUGIN_VERSION "${PLUGIN_VERSION}")

add_definitions(-DPLUGIN_VERSION="${PLUGIN_VERSION}")
add_definitions(-DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1)
add_definitions(-DGLEW_BUILD=GLEW_STATIC)
if(WIN32)
	add_definitions(-DAPL=0 -DIBM=1 -DLIN=0 -D_WIN32_WINNT=0x0600)
elseif(APPLE)
	add_definitions(-DAPL=1 -DIBM=0 -DLIN=0)
else()
	add_definitions(-DAPL=0 -DIBM=0 -DLIN=1)
endif()

# linking
find_library(LIBACFUTILS_LIBRARY acfutils "${LIBACFUTILS}/qmake/${PLAT_SHORT}")
if(WIN32)
	find_library(XPLM_LIBRARY XPLM_64 "${LIBACFUTILS}/SDK/Libraries/Win")
	find_library(XPWIDGETS_LIBRARY XPWidgets_64
	    "${LIBACFUTILS}/SDK/Libraries/Win")
	find_library(OPENGL_LIBRARY opengl32 "../GL_for_Windows/lib")
	target_link_libraries(openwxr
	    ${LIBACFUTILS_LIBRARY}
	    ${XPLM_LIBRARY}
	    ${XPWIDGETS_LIBRARY}
	    ${OPENGL_LIBRARY}
	    ${DEP_LIBS}
	)
	set_target_properties(openwxr PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -undefined_warning \
	    -fvisibility=hidden -static-libgcc -static-libstdc++")
elseif(APPLE)
	find_library(XPLM_LIBRARY XPLM "${LIBACFUTILS}/SDK/Libraries/Mac")
	find_library(XPWIDGETS_LIBRARY XPWidgets
	    "${LIBACFUTILS}/SDK/Libraries/Mac")
	find_library(OPENGL_FRAMEWORK OpenGL)
	target_link_libraries(openwxr
	    ${LIBACFUTILS_LIBRARY}
	    ${XPLM_LIBRARY}
	    ${XPWIDGETS_LIBRARY}
	    ${OPENGL_FRAMEWORK}
	    ${DEP_LIBS}
	)
	set_target_properties(openwxr PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -fvisibility=hidden -bundle")
else()
	target_link_libraries(openwxr
	    ${LIBACFUTILS_LIBRARY}
	    ${DEP_LIBS}
	)
	set_target_properties(openwxr PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
	    -undefined_warning -fPIC -fvisibility=hidden \
	    -static-libgcc -static-libstdc++ \
	    -Wl,--version-script=symbols.version")
endif()

set_target_properties(openwxr PROPERTIES PREFIX "")
set_target_properties(openwxr PROPERTIES SUFFIX "")
set_target_properties(openwxr PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(openwxr PROPERTIES C_STANDARD 11)
set_target_properties(openwxr PROPERTIES CXX_STANDARD 17)

set_target_properties(openwxr PROPERTIES RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/../${PLUGIN_BIN_OUTDIR}")
set_target_properties(openwxr PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/../${PLUGIN_BIN_OUTDIR}")
set_target_properties(openwxr PROPERTIES OUTPUT_NAME "OpenWXR.xpl")
