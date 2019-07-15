cmake_minimum_required( VERSION 2.8 )

project( AA-SIPP-m )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )
set( CMAKE_BUILD_TYPE Debug)

include_directories(
	include
)

set( SOURCE_FILES 
     src/main.cpp
	 src/tinyxml2.cpp
	 src/xmlLogger.cpp
	 src/mission.cpp
	 src/map.cpp
	 src/task.cpp
	 src/config.cpp
	 src/dynamicobstacles.cpp
	 src/aa_sipp.cpp
	 src/constraints.cpp )

#set( HEADER_FILES
#     tinyxml2.h
#     searchresult.h
#     gl_const.h
#     xmlLogger.h
#     mission.h
#     map.h
#	 task.h
#	 dynamicobstacles.h
#     config.h
#     aa_sipp.h
#     structs.h
#     constraints.h )

#add_executable( AA-SIPP-m ${SOURCE_FILES})
add_library( sipp ${SOURCE_FILES} )