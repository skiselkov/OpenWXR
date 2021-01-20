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

# Copyright 2018 Saso Kiselkov. All rights reserved.

cmake_minimum_required(VERSION 2.8)
project(openwxr C)

SET(SRC
    atmo_xp11.c
    dbg_log.c
    fontmgr.c
    standalone.c
    wxr.c
    xplane.c)
SET(HDR
    atmo.h
    atmo_xp11.h
    dbg_log.h
    fontmgr.h
    standalone.h
    wxr.h
    xplane.h)

SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} -DDEBUG -g")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG -O0 -g")

SET(ALL_SRC ${SRC} ${HDR})
LIST(SORT ALL_SRC)

if(APPLE)
	add_executable(openwxr ${ALL_SRC})
else()
	add_library(openwxr SHARED ${ALL_SRC})
endif()

include_directories(openwxr PUBLIC
    "${LIBACFUTILS}/src"
    "${LIBACFUTILS}/glew/glew-1.13.0-linux-64/install/include"
    "${LIBACFUTILS}/cglm/cglm-0.4.1/include"
    "../api"
    "${OPENGPWS}/api")

#compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror --std=c99 \
    -Wno-unused-local-typedefs -Wno-missing-field-initializers")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
    -DCHECK_RESULT_USED=\"__attribute__ ((warn_unused_result))\"")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_GNU_SOURCE -D_FILE_OFFSET_BITS=64")

if(${FAST_DEBUG})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DFAST_DEBUG=${FAST_DEBUG}")
endif()

#libraries
include_directories(openwxr PUBLIC
    "${LIBACFUTILS}/SDK/CHeaders/XPLM"
    "${LIBACFUTILS}/SDK/CHeaders/Widgets"
    "${LIBACFUTILS}/SDK/CHeaders/Wrappers")

if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -arch x86_64 -Wno-deprecated")
endif()

# Yes, we need to strip a trailing newline from command output. CMake is
# "smart" like that...
execute_process(COMMAND git rev-parse --short HEAD
    OUTPUT_VARIABLE PLUGIN_VERSION)
string(REGEX REPLACE "\n$" "" PLUGIN_VERSION "${PLUGIN_VERSION}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPLUGIN_VERSION='\"${PLUGIN_VERSION}\"'")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 \
    -DXPLM301=1 -DGLEW_BUILD=GLEW_STATIC")
if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=1 -DIBM=0 -DLIN=0")
else()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=0 -DIBM=0 -DLIN=1")
endif()

if(APPLE)
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps mac-64 --cflags
	    OUTPUT_VARIABLE DEP_CFLAGS)
else()
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps linux-64 --cflags
	    OUTPUT_VARIABLE DEP_CFLAGS)
endif()
string(REGEX REPLACE "\n$" "" DEP_CFLAGS "${DEP_CFLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${DEP_CFLAGS}")

# linking
# Careful: order matters in target_link_libraries. Dependencies must be
# ordered as first symbol callers and then as symbol providers (i.e.
# libpng16 depends on libz, so linking must be first: libpng, then libz).
link_libraries(m)
if(APPLE)
	set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH}
	    ${LIBACFUTILS}/SDK/Libraries/Mac
	    ${LIBACFUTILS}/qmake/mac64
	    ${LIBACFUTILS}/opus/opusfile-mac-64/lib
	    ${LIBACFUTILS}/opus/opus-mac-64/lib
	    ${LIBACFUTILS}/opus/libogg-mac-64/install/lib
	    ${LIBACFUTILS}/opus/libogg-mac-64/install/lib
	    ${LIBACFUTILS}/freetype/freetype-mac-64/lib
	    ${LIBACFUTILS}/cairo/pixman-mac-64/lib
	    ${LIBACFUTILS}/cairo/cairo-mac-64/lib
	    ${LIBACFUTILS}/libpng/libpng-mac-64/lib
	    ${LIBACFUTILS}/zlib/zlib-mac-64/lib
	    ${LIBACFUTILS}/lzma/qmake/mac-64)
	FIND_LIBRARY(XPLM_LIBRARY XPLM)
	FIND_LIBRARY(XPWIDGETS_LIBRARY XPWidgets)
	FIND_LIBRARY(OPENGL_LIBRARY OpenGL)
	FIND_LIBRARY(OPENAL_LIBRARY OpenAL)
	FIND_LIBRARY(LIBACFUTILS_LIBRARY acfutils)
	FIND_LIBRARY(OPUSFILE_LIBRARY opusfile)
	FIND_LIBRARY(OPUS_LIBRARY opus)
	FIND_LIBRARY(OGG_LIBRARY ogg)
	FIND_LIBRARY(FREETYPE_LIBRARY freetype)
	FIND_LIBRARY(PIXMAN_LIBRARY pixman-1)
	FIND_LIBRARY(CAIRO_LIBRARY cairo)
	FIND_LIBRARY(LZMA_LIBRARY lzma)
	FIND_LIBRARY(LIBPNG_LIBRARY png16)
	FIND_LIBRARY(ZLIB_LIBRARY z)
	target_link_libraries(openwxr
	    ${LIBACFUTILS_LIBRARY}
	    ${OPENAL_LIBRARY}
	    ${OPENGL_LIBRARY}
	    ${XPLM_LIBRARY}
	    ${XPWIDGETS_LIBRARY}
	    ${OPUSFILE_LIBRARY}
	    ${OPUS_LIBRARY}
	    ${OGG_LIBRARY}
	    ${CAIRO_LIBRARY}
	    ${PIXMAN_LIBRARY}
	    ${FREETYPE_LIBRARY}
	    ${LIBPNG_LIBRARY}
	    ${LZMA_LIBRARY}
	    ${ZLIB_LIBRARY}
	    )
else()
	set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH}
	    ${LIBACFUTILS}/qmake/lin64
	    ${LIBACFUTILS}/opus/opusfile-linux-64/lib
	    ${LIBACFUTILS}/opus/opus-linux-64/lib
	    ${LIBACFUTILS}/opus/libogg-linux-64/install/lib
	    ${LIBACFUTILS}/freetype/freetype-linux-64/lib
	    ${LIBACFUTILS}/cairo/pixman-linux-64/lib
	    ${LIBACFUTILS}/cairo/cairo-linux-64/lib
	    ${LIBACFUTILS}/libpng/libpng-linux-64/lib
	    ${LIBACFUTILS}/zlib/zlib-linux-64/lib
	    ${LIBACFUTILS}/lzma/qmake/linux-64)
	FIND_LIBRARY(LIBACFUTILS_LIBRARY acfutils)
	FIND_LIBRARY(OPUSFILE_LIBRARY opusfile)
	FIND_LIBRARY(OPUS_LIBRARY opus)
	FIND_LIBRARY(OGG_LIBRARY ogg)
	FIND_LIBRARY(FREETYPE_LIBRARY freetype)
	FIND_LIBRARY(PIXMAN_LIBRARY pixman-1)
	FIND_LIBRARY(CAIRO_LIBRARY cairo)
	FIND_LIBRARY(LIBPNG_LIBRARY png16)
	FIND_LIBRARY(ZLIB_LIBRARY z ${LIBACFUTILS}/zlib/zlib-linux-64/lib)
	FIND_LIBRARY(LZMA_LIBRARY lzma)
	FIND_LIBRARY(LIBC_NONSHARED c_nonshared)
	target_link_libraries(openwxr
	    ${LIBACFUTILS_LIBRARY}
	    ${OPUSFILE_LIBRARY}
	    ${OPUS_LIBRARY}
	    ${OGG_LIBRARY}
	    ${CAIRO_LIBRARY}
	    ${PIXMAN_LIBRARY}
	    ${FREETYPE_LIBRARY}
	    ${LIBPNG_LIBRARY}
	    # Need to forcibly shove this down cmake's throat
	    "${LIBACFUTILS}/zlib/zlib-linux-64/lib/libz.a"
	    "${LIBACFUTILS}/glew/glew-1.13.0-linux-64/install/lib64/libGLEWmx.a"
	    ${LIBC_NONSHARED}
	    ${LZMA_LIBRARY}
	    )
endif()

SET_TARGET_PROPERTIES(openwxr PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES(openwxr PROPERTIES SUFFIX "")

if(APPLE)
	set(OUTPUT_FILENAME "mac.xpl")
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps mac-64 --libs
	    OUTPUT_VARIABLE LIBACFUTILS_DEP_LDFLAGS)
	SET_TARGET_PROPERTIES(openwxr PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -fPIC -fvisibility=hidden -bundle")
else()
	set(OUTPUT_FILENAME "lin.xpl")
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps linux-64 --libs
	    OUTPUT_VARIABLE LIBACFUTILS_DEP_LDFLAGS)
	SET_TARGET_PROPERTIES(openwxr PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
	    -undefined_warnsing -fPIC -fvisibility=hidden \
	    ${LIBACFUTILS}/zlib/zlib-linux-64/lib/libz.a")
endif()

set_target_properties(openwxr PROPERTIES RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(openwxr PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(openwxr PROPERTIES OUTPUT_NAME "${OUTPUT_FILENAME}")
