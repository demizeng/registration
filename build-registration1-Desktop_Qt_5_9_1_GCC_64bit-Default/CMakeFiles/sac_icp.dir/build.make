# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vless/projects/qt_projs/registration/registration1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vless/projects/qt_projs/registration/build-registration1-Desktop_Qt_5_9_1_GCC_64bit-Default

# Include any dependencies generated for this target.
include CMakeFiles/sac_icp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sac_icp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sac_icp.dir/flags.make

CMakeFiles/sac_icp.dir/sac-icp.cpp.o: CMakeFiles/sac_icp.dir/flags.make
CMakeFiles/sac_icp.dir/sac-icp.cpp.o: /home/vless/projects/qt_projs/registration/registration1/sac-icp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vless/projects/qt_projs/registration/build-registration1-Desktop_Qt_5_9_1_GCC_64bit-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sac_icp.dir/sac-icp.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sac_icp.dir/sac-icp.cpp.o -c /home/vless/projects/qt_projs/registration/registration1/sac-icp.cpp

CMakeFiles/sac_icp.dir/sac-icp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sac_icp.dir/sac-icp.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vless/projects/qt_projs/registration/registration1/sac-icp.cpp > CMakeFiles/sac_icp.dir/sac-icp.cpp.i

CMakeFiles/sac_icp.dir/sac-icp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sac_icp.dir/sac-icp.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vless/projects/qt_projs/registration/registration1/sac-icp.cpp -o CMakeFiles/sac_icp.dir/sac-icp.cpp.s

CMakeFiles/sac_icp.dir/sac-icp.cpp.o.requires:

.PHONY : CMakeFiles/sac_icp.dir/sac-icp.cpp.o.requires

CMakeFiles/sac_icp.dir/sac-icp.cpp.o.provides: CMakeFiles/sac_icp.dir/sac-icp.cpp.o.requires
	$(MAKE) -f CMakeFiles/sac_icp.dir/build.make CMakeFiles/sac_icp.dir/sac-icp.cpp.o.provides.build
.PHONY : CMakeFiles/sac_icp.dir/sac-icp.cpp.o.provides

CMakeFiles/sac_icp.dir/sac-icp.cpp.o.provides.build: CMakeFiles/sac_icp.dir/sac-icp.cpp.o


# Object files for target sac_icp
sac_icp_OBJECTS = \
"CMakeFiles/sac_icp.dir/sac-icp.cpp.o"

# External object files for target sac_icp
sac_icp_EXTERNAL_OBJECTS =

sac_icp: CMakeFiles/sac_icp.dir/sac-icp.cpp.o
sac_icp: CMakeFiles/sac_icp.dir/build.make
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_system.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_regex.so
sac_icp: /usr/lib/x86_64-linux-gnu/libpthread.so
sac_icp: /usr/local/lib/libpcl_common.so
sac_icp: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
sac_icp: /usr/local/lib/libpcl_kdtree.so
sac_icp: /usr/local/lib/libpcl_octree.so
sac_icp: /usr/local/lib/libpcl_search.so
sac_icp: /usr/local/lib/libpcl_sample_consensus.so
sac_icp: /usr/local/lib/libpcl_filters.so
sac_icp: /usr/lib/libOpenNI2.so
sac_icp: /usr/local/lib/libpcl_io.so
sac_icp: /usr/local/lib/libpcl_features.so
sac_icp: /usr/local/lib/libpcl_ml.so
sac_icp: /usr/local/lib/libpcl_segmentation.so
sac_icp: /usr/local/lib/libpcl_tracking.so
sac_icp: /usr/local/lib/libpcl_visualization.so
sac_icp: /usr/lib/x86_64-linux-gnu/libqhull.so
sac_icp: /usr/local/lib/libpcl_surface.so
sac_icp: /usr/local/lib/libpcl_registration.so
sac_icp: /usr/local/lib/libpcl_keypoints.so
sac_icp: /usr/local/lib/libpcl_recognition.so
sac_icp: /usr/local/lib/libpcl_stereo.so
sac_icp: /usr/local/lib/libpcl_cuda_segmentation.so
sac_icp: /usr/local/lib/libpcl_cuda_features.so
sac_icp: /usr/local/lib/libpcl_cuda_sample_consensus.so
sac_icp: /usr/local/lib/libpcl_outofcore.so
sac_icp: /usr/local/lib/libpcl_people.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_system.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
sac_icp: /usr/lib/x86_64-linux-gnu/libboost_regex.so
sac_icp: /usr/lib/x86_64-linux-gnu/libpthread.so
sac_icp: /usr/lib/x86_64-linux-gnu/libqhull.so
sac_icp: /usr/lib/libOpenNI2.so
sac_icp: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
sac_icp: /usr/lib/libvtkGenericFiltering.so.5.8.0
sac_icp: /usr/lib/libvtkGeovis.so.5.8.0
sac_icp: /usr/lib/libvtkCharts.so.5.8.0
sac_icp: /usr/local/lib/libpcl_common.so
sac_icp: /usr/local/lib/libpcl_kdtree.so
sac_icp: /usr/local/lib/libpcl_octree.so
sac_icp: /usr/local/lib/libpcl_search.so
sac_icp: /usr/local/lib/libpcl_sample_consensus.so
sac_icp: /usr/local/lib/libpcl_filters.so
sac_icp: /usr/local/lib/libpcl_io.so
sac_icp: /usr/local/lib/libpcl_features.so
sac_icp: /usr/local/lib/libpcl_ml.so
sac_icp: /usr/local/lib/libpcl_segmentation.so
sac_icp: /usr/local/lib/libpcl_tracking.so
sac_icp: /usr/local/lib/libpcl_visualization.so
sac_icp: /usr/local/lib/libpcl_surface.so
sac_icp: /usr/local/lib/libpcl_registration.so
sac_icp: /usr/local/lib/libpcl_keypoints.so
sac_icp: /usr/local/lib/libpcl_recognition.so
sac_icp: /usr/local/lib/libpcl_stereo.so
sac_icp: /usr/local/lib/libpcl_cuda_segmentation.so
sac_icp: /usr/local/lib/libpcl_cuda_features.so
sac_icp: /usr/local/lib/libpcl_cuda_sample_consensus.so
sac_icp: /usr/local/lib/libpcl_outofcore.so
sac_icp: /usr/local/lib/libpcl_people.so
sac_icp: /usr/lib/libvtkViews.so.5.8.0
sac_icp: /usr/lib/libvtkInfovis.so.5.8.0
sac_icp: /usr/lib/libvtkWidgets.so.5.8.0
sac_icp: /usr/lib/libvtkVolumeRendering.so.5.8.0
sac_icp: /usr/lib/libvtkHybrid.so.5.8.0
sac_icp: /usr/lib/libvtkParallel.so.5.8.0
sac_icp: /usr/lib/libvtkRendering.so.5.8.0
sac_icp: /usr/lib/libvtkImaging.so.5.8.0
sac_icp: /usr/lib/libvtkGraphics.so.5.8.0
sac_icp: /usr/lib/libvtkIO.so.5.8.0
sac_icp: /usr/lib/libvtkFiltering.so.5.8.0
sac_icp: /usr/lib/libvtkCommon.so.5.8.0
sac_icp: /usr/lib/libvtksys.so.5.8.0
sac_icp: CMakeFiles/sac_icp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vless/projects/qt_projs/registration/build-registration1-Desktop_Qt_5_9_1_GCC_64bit-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sac_icp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sac_icp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sac_icp.dir/build: sac_icp

.PHONY : CMakeFiles/sac_icp.dir/build

CMakeFiles/sac_icp.dir/requires: CMakeFiles/sac_icp.dir/sac-icp.cpp.o.requires

.PHONY : CMakeFiles/sac_icp.dir/requires

CMakeFiles/sac_icp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sac_icp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sac_icp.dir/clean

CMakeFiles/sac_icp.dir/depend:
	cd /home/vless/projects/qt_projs/registration/build-registration1-Desktop_Qt_5_9_1_GCC_64bit-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vless/projects/qt_projs/registration/registration1 /home/vless/projects/qt_projs/registration/registration1 /home/vless/projects/qt_projs/registration/build-registration1-Desktop_Qt_5_9_1_GCC_64bit-Default /home/vless/projects/qt_projs/registration/build-registration1-Desktop_Qt_5_9_1_GCC_64bit-Default /home/vless/projects/qt_projs/registration/build-registration1-Desktop_Qt_5_9_1_GCC_64bit-Default/CMakeFiles/sac_icp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sac_icp.dir/depend

