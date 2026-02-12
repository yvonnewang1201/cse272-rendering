find_path(EMBREE_INCLUDE_PATH embree4/rtcore.h
  ${CMAKE_SOURCE_DIR}/embree/include
  /usr/include
  /usr/local/include
  /opt/local/include)

if (APPLE)
    if ("arm64" STREQUAL CMAKE_SYSTEM_PROCESSOR)
    set(EMBREE_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/embree/lib-macos/arm64)
    message("-- Detected Apple Silicon (using Embree for arm64)")
    else ()
    set(EMBREE_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/embree/lib-macos)
    endif ()
find_library(EMBREE_LIBRARY NAMES embree4 PATHS
    ${EMBREE_LIBRARY_DIR}
  /usr/lib
  /usr/local/lib
  /opt/local/lib)
elseif (WIN32)
find_library(EMBREE_LIBRARY NAMES embree4 PATHS
  ${CMAKE_SOURCE_DIR}/embree/lib-win32)
else ()
find_library(EMBREE_LIBRARY NAMES embree4 PATHS
  ${CMAKE_SOURCE_DIR}/embree/lib-linux
  /usr/lib
  /usr/local/lib
  /opt/local/lib)
endif ()

if (EMBREE_INCLUDE_PATH AND EMBREE_LIBRARY)
  set(EMBREE_FOUND TRUE)
endif ()
