cmake_minimum_required(VERSION 3.5)
project(indi_rollogo C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")
include(GNUInstallDirs)

find_package(INDI REQUIRED)
link_directories(/usr/local/lib)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})
include_directories(${WIRINGPI_INCLUDE_DIR})

include(CMakeCommon)

set(INDI_ROLLOGO_VERSION_MAJOR 1)
set(INDI_ROLLOGO_VERSION_MINOR 1)


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_rollogo.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_rollogo.xml)

IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 6.3.9)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat-overflow=2")
ENDIF ()

############### Rolloff LOGO! ####################
set(indirollogo_SRCS
   ${CMAKE_CURRENT_SOURCE_DIR}/rollogo.cpp
   )

add_executable(indi_rollogo ${indirollogo_SRCS})
target_link_libraries(indi_rollogo ${INDI_LIBRARIES} crypt modbus)

install(TARGETS indi_rollogo RUNTIME DESTINATION bin )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_rollogo.xml DESTINATION ${INDI_DATA_DIR})
