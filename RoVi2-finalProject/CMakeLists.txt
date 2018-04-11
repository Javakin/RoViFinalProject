#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)


# The name of the project.
PROJECT(ObjectAvoidance)

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_VERBOSE_MAKEFILE False)

# Now set the RW/RWS root (edit this if necessary)
SET(RW_ROOT $ENV{RW_ROOT})
SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})


# Set build type to release
IF( NOT CMAKE_BUILD_TYPE )
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
MESSAGE("-- RobWork root: " ${RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake)
MESSAGE("-- RobWork_DIR:" ${RobWork_DIR})
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Use RobWorkStudio
MESSAGE("-- RobWorkStudio root: " ${RWSTUDIO_ROOT})
SET(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
MESSAGE("-- RobWorkStudio_DIR:" ${RobWorkStudio_DIR})
FIND_PACKAGE(RobWorkStudio REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

# Use OpenCV 3
find_package(OpenCV 3 REQUIRED)

# Set the output dir for generated libraries and binaries
IF(MSVC)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
ELSE()
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
ENDIF()

# if we want to use ui files add them here
SET(UIS_FILES )

IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ELSE()
	QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ENDIF()

#Need to add the current binary dir to the include directory because the generated source files are placed here
include_directories(${CMAKE_CURRENT_BINARY_DIR})


# MOC
IF(RWS_USE_QT5)
	QT5_WRAP_CPP(MocSrcFiles src/ObjectAvoidance.hpp src/ip.h)
ELSE()
	QT4_WRAP_CPP(MocSrcFiles src/ObjectAvoidance.hpp src/ip.h)
ENDIF()

# Rcc the files:
IF(RWS_USE_QT5)
	QT5_ADD_RESOURCES(RccSrcFiles src/resources.qrc)
ELSE()
	QT4_ADD_RESOURCES(RccSrcFiles src/resources.qrc)
ENDIF()

# Add the source files to build
SET(SrcFiles src/ObjectAvoidance.cpp src/ip.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(ObjectAvoidance MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(ObjectAvoidance ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${OpenCV_LIBS})

MESSAGE("-- " ${PROJECT_NAME} " done!")

