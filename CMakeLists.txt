##---------------------------------------------------------------------------------------------------------------------
##  VCAL - Visual Control Abstraction Layer
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
project(vcal VERSION 0.1 LANGUAGES C CXX)

##########################################
######		Configure project		######
##########################################
set(BUILD_EXAMPLES ON)

list(APPEND VCAL_DEFINITIONS "")
#########################################
###### Loading 3rd party libraries.######
#########################################
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

#########################################
## Load opencv
find_package(OpenCV 3 REQUIRED core highgui imgproc imgcodecs features2d xfeatures2d calib3d video videoio)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

#########################################
# rgbd_tools
find_package(rgbd_tools REQUIRED)

#########################################
# fastcom
find_package(fastcom)
if(${fastcom_FOUND})
	list(APPEND VCAL_DEFINITIONS "HAS_FASTCOM")
endif(${fastcom_FOUND})

#########################################
# catkin
find_package(catkin REQUIRED roscpp cv_bridge image_transport sensor_msgs)
if(${catkin_FOUND})
	catkin_package(
		CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs
		INCLUDE_DIRS include
	)
	list(APPEND VCAL_DEFINITIONS "HAS_ROS")
endif(${catkin_FOUND})

#########################################
######			Library 		   ######
#########################################
#add_definitions(-fopenmp)

# Collect all sources
file(GLOB_RECURSE VCAL_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE VCAL_HEADER_FILES "include/*.h" "include/*.inl")
 
add_library(${PROJECT_NAME} SHARED ${VCAL_SOURCE_FILES} ${VCAL_HEADER_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include) 

target_compile_definitions(${PROJECT_NAME} PUBLIC ${VCAL_DEFINITIONS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_link_libraries(${PROJECT_NAME} PUBLIC rgbd_tools::rgbd_tools)

if(${fastcom_FOUND})
	target_link_libraries(${PROJECT_NAME} PUBLIC fastcom::fastcom)
endif(${fastcom_FOUND})


if(${catkin_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${catkin_LIBRARIES} )
	add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif(${catkin_FOUND})

#########################################
######			Documentation	   ######
#########################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(	doc_vcal
						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
						COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)

#########################################
######			INSTALL 		   ######
#########################################
include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

install(TARGETS vcal
        EXPORT vcalTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )

write_basic_package_version_file(
    vcalConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )


# This installs the include folder
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

export(TARGETS vcal NAMESPACE vcal:: FILE vcalTargets.cmake)

# This generates vcalTargets.cmake
install(EXPORT vcalTargets
    FILE vcalTargets.cmake
    NAMESPACE vcal::
    DESTINATION lib/cmake/vcal
    )

configure_file(vcalConfig.cmake.in vcalConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/vcalConfig.cmake"
        DESTINATION lib/cmake/vcal
        )



# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      IMMEDIATE @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 
 

#########################################
######			Examples 		   ######
#########################################
if(${BUILD_EXAMPLES})
    add_executable(example_mono examples/MonoVisualControl.cpp)
    target_link_libraries(example_mono PRIVATE vcal)

    add_executable(example_depth examples/DepthVisualControl.cpp)
    target_link_libraries(example_depth PRIVATE vcal)
endif(${BUILD_EXAMPLES})
