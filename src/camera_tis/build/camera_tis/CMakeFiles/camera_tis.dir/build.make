# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/firefly/test_mysunny/src/camera_tis

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/firefly/test_mysunny/src/camera_tis/build/camera_tis

# Include any dependencies generated for this target.
include CMakeFiles/camera_tis.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera_tis.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera_tis.dir/flags.make

CMakeFiles/camera_tis.dir/src/camera_tis.cpp.o: CMakeFiles/camera_tis.dir/flags.make
CMakeFiles/camera_tis.dir/src/camera_tis.cpp.o: ../../src/camera_tis.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/firefly/test_mysunny/src/camera_tis/build/camera_tis/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/camera_tis.dir/src/camera_tis.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera_tis.dir/src/camera_tis.cpp.o -c /home/firefly/test_mysunny/src/camera_tis/src/camera_tis.cpp

CMakeFiles/camera_tis.dir/src/camera_tis.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_tis.dir/src/camera_tis.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/firefly/test_mysunny/src/camera_tis/src/camera_tis.cpp > CMakeFiles/camera_tis.dir/src/camera_tis.cpp.i

CMakeFiles/camera_tis.dir/src/camera_tis.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_tis.dir/src/camera_tis.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/firefly/test_mysunny/src/camera_tis/src/camera_tis.cpp -o CMakeFiles/camera_tis.dir/src/camera_tis.cpp.s

CMakeFiles/camera_tis.dir/src/E2camData.cpp.o: CMakeFiles/camera_tis.dir/flags.make
CMakeFiles/camera_tis.dir/src/E2camData.cpp.o: ../../src/E2camData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/firefly/test_mysunny/src/camera_tis/build/camera_tis/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/camera_tis.dir/src/E2camData.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera_tis.dir/src/E2camData.cpp.o -c /home/firefly/test_mysunny/src/camera_tis/src/E2camData.cpp

CMakeFiles/camera_tis.dir/src/E2camData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_tis.dir/src/E2camData.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/firefly/test_mysunny/src/camera_tis/src/E2camData.cpp > CMakeFiles/camera_tis.dir/src/E2camData.cpp.i

CMakeFiles/camera_tis.dir/src/E2camData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_tis.dir/src/E2camData.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/firefly/test_mysunny/src/camera_tis/src/E2camData.cpp -o CMakeFiles/camera_tis.dir/src/E2camData.cpp.s

CMakeFiles/camera_tis.dir/src/FileOut.cpp.o: CMakeFiles/camera_tis.dir/flags.make
CMakeFiles/camera_tis.dir/src/FileOut.cpp.o: ../../src/FileOut.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/firefly/test_mysunny/src/camera_tis/build/camera_tis/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/camera_tis.dir/src/FileOut.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera_tis.dir/src/FileOut.cpp.o -c /home/firefly/test_mysunny/src/camera_tis/src/FileOut.cpp

CMakeFiles/camera_tis.dir/src/FileOut.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_tis.dir/src/FileOut.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/firefly/test_mysunny/src/camera_tis/src/FileOut.cpp > CMakeFiles/camera_tis.dir/src/FileOut.cpp.i

CMakeFiles/camera_tis.dir/src/FileOut.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_tis.dir/src/FileOut.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/firefly/test_mysunny/src/camera_tis/src/FileOut.cpp -o CMakeFiles/camera_tis.dir/src/FileOut.cpp.s

# Object files for target camera_tis
camera_tis_OBJECTS = \
"CMakeFiles/camera_tis.dir/src/camera_tis.cpp.o" \
"CMakeFiles/camera_tis.dir/src/E2camData.cpp.o" \
"CMakeFiles/camera_tis.dir/src/FileOut.cpp.o"

# External object files for target camera_tis
camera_tis_EXTERNAL_OBJECTS =

libcamera_tis.so: CMakeFiles/camera_tis.dir/src/camera_tis.cpp.o
libcamera_tis.so: CMakeFiles/camera_tis.dir/src/E2camData.cpp.o
libcamera_tis.so: CMakeFiles/camera_tis.dir/src/FileOut.cpp.o
libcamera_tis.so: CMakeFiles/camera_tis.dir/build.make
libcamera_tis.so: /opt/ros/galactic/lib/libcomponent_manager.so
libcamera_tis.so: /opt/ros/galactic/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libsensor_msgs__rosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libsensor_msgs__rosidl_typesupport_cpp.so
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_face.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_hfs.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_quality.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_tracking.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.2.0
libcamera_tis.so: /opt/ros/galactic/lib/librclcpp.so
libcamera_tis.so: /opt/ros/galactic/lib/liblibstatistics_collector.so
libcamera_tis.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
libcamera_tis.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/librcl.so
libcamera_tis.so: /opt/ros/galactic/lib/librmw_implementation.so
libcamera_tis.so: /opt/ros/galactic/lib/librcl_logging_spdlog.so
libcamera_tis.so: /opt/ros/galactic/lib/librcl_logging_interface.so
libcamera_tis.so: /opt/ros/galactic/lib/librcl_yaml_param_parser.so
libcamera_tis.so: /opt/ros/galactic/lib/librmw.so
libcamera_tis.so: /opt/ros/galactic/lib/libyaml.so
libcamera_tis.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_generator_c.so
libcamera_tis.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_generator_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libtracetools.so
libcamera_tis.so: /opt/ros/galactic/lib/libament_index_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libclass_loader.so
libcamera_tis.so: /opt/ros/galactic/lib/aarch64-linux-gnu/libconsole_bridge.so.1.0
libcamera_tis.so: /opt/ros/galactic/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libcomposition_interfaces__rosidl_generator_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libcomposition_interfaces__rosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_generator_c.so
libcamera_tis.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libsensor_msgs__rosidl_generator_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_generator_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_generator_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_generator_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/librosidl_typesupport_introspection_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/librosidl_typesupport_introspection_c.so
libcamera_tis.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/librosidl_typesupport_cpp.so
libcamera_tis.so: /opt/ros/galactic/lib/librosidl_typesupport_c.so
libcamera_tis.so: /opt/ros/galactic/lib/librcpputils.so
libcamera_tis.so: /opt/ros/galactic/lib/librosidl_runtime_c.so
libcamera_tis.so: /opt/ros/galactic/lib/librcutils.so
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libmodbus.so
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_text.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.2.0
libcamera_tis.so: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.2.0
libcamera_tis.so: CMakeFiles/camera_tis.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/firefly/test_mysunny/src/camera_tis/build/camera_tis/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libcamera_tis.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera_tis.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera_tis.dir/build: libcamera_tis.so

.PHONY : CMakeFiles/camera_tis.dir/build

CMakeFiles/camera_tis.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera_tis.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera_tis.dir/clean

CMakeFiles/camera_tis.dir/depend:
	cd /home/firefly/test_mysunny/src/camera_tis/build/camera_tis && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/firefly/test_mysunny/src/camera_tis /home/firefly/test_mysunny/src/camera_tis /home/firefly/test_mysunny/src/camera_tis/build/camera_tis /home/firefly/test_mysunny/src/camera_tis/build/camera_tis /home/firefly/test_mysunny/src/camera_tis/build/camera_tis/CMakeFiles/camera_tis.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera_tis.dir/depend
