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
CMAKE_SOURCE_DIR = /home/lijialiang/xbotcon

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lijialiang/xbotcon/build

# Include any dependencies generated for this target.
include CMakeFiles/XBOTCON.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/XBOTCON.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/XBOTCON.dir/flags.make

CMakeFiles/XBOTCON.dir/main.cpp.o: CMakeFiles/XBOTCON.dir/flags.make
CMakeFiles/XBOTCON.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lijialiang/xbotcon/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/XBOTCON.dir/main.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/XBOTCON.dir/main.cpp.o -c /home/lijialiang/xbotcon/main.cpp

CMakeFiles/XBOTCON.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/XBOTCON.dir/main.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lijialiang/xbotcon/main.cpp > CMakeFiles/XBOTCON.dir/main.cpp.i

CMakeFiles/XBOTCON.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/XBOTCON.dir/main.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lijialiang/xbotcon/main.cpp -o CMakeFiles/XBOTCON.dir/main.cpp.s

CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.o: CMakeFiles/XBOTCON.dir/flags.make
CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.o: ../src/distancedetection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lijialiang/xbotcon/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.o -c /home/lijialiang/xbotcon/src/distancedetection.cpp

CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lijialiang/xbotcon/src/distancedetection.cpp > CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.i

CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lijialiang/xbotcon/src/distancedetection.cpp -o CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.s

CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.o: CMakeFiles/XBOTCON.dir/flags.make
CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.o: ../src/kalman_filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lijialiang/xbotcon/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.o -c /home/lijialiang/xbotcon/src/kalman_filter.cpp

CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lijialiang/xbotcon/src/kalman_filter.cpp > CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.i

CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lijialiang/xbotcon/src/kalman_filter.cpp -o CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.s

CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.o: CMakeFiles/XBOTCON.dir/flags.make
CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.o: ../src/ImgPreProcess.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lijialiang/xbotcon/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.o -c /home/lijialiang/xbotcon/src/ImgPreProcess.cpp

CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lijialiang/xbotcon/src/ImgPreProcess.cpp > CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.i

CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lijialiang/xbotcon/src/ImgPreProcess.cpp -o CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.s

CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.o: CMakeFiles/XBOTCON.dir/flags.make
CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.o: ../src/objClassifier.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lijialiang/xbotcon/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.o -c /home/lijialiang/xbotcon/src/objClassifier.cpp

CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lijialiang/xbotcon/src/objClassifier.cpp > CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.i

CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lijialiang/xbotcon/src/objClassifier.cpp -o CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.s

CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.o: CMakeFiles/XBOTCON.dir/flags.make
CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.o: ../src/getTarget2dPosition.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lijialiang/xbotcon/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.o -c /home/lijialiang/xbotcon/src/getTarget2dPosition.cpp

CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lijialiang/xbotcon/src/getTarget2dPosition.cpp > CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.i

CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lijialiang/xbotcon/src/getTarget2dPosition.cpp -o CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.s

# Object files for target XBOTCON
XBOTCON_OBJECTS = \
"CMakeFiles/XBOTCON.dir/main.cpp.o" \
"CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.o" \
"CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.o" \
"CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.o" \
"CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.o" \
"CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.o"

# External object files for target XBOTCON
XBOTCON_EXTERNAL_OBJECTS =

XBOTCON: CMakeFiles/XBOTCON.dir/main.cpp.o
XBOTCON: CMakeFiles/XBOTCON.dir/src/distancedetection.cpp.o
XBOTCON: CMakeFiles/XBOTCON.dir/src/kalman_filter.cpp.o
XBOTCON: CMakeFiles/XBOTCON.dir/src/ImgPreProcess.cpp.o
XBOTCON: CMakeFiles/XBOTCON.dir/src/objClassifier.cpp.o
XBOTCON: CMakeFiles/XBOTCON.dir/src/getTarget2dPosition.cpp.o
XBOTCON: CMakeFiles/XBOTCON.dir/build.make
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
XBOTCON: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
XBOTCON: CMakeFiles/XBOTCON.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lijialiang/xbotcon/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable XBOTCON"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/XBOTCON.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/XBOTCON.dir/build: XBOTCON

.PHONY : CMakeFiles/XBOTCON.dir/build

CMakeFiles/XBOTCON.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/XBOTCON.dir/cmake_clean.cmake
.PHONY : CMakeFiles/XBOTCON.dir/clean

CMakeFiles/XBOTCON.dir/depend:
	cd /home/lijialiang/xbotcon/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lijialiang/xbotcon /home/lijialiang/xbotcon /home/lijialiang/xbotcon/build /home/lijialiang/xbotcon/build /home/lijialiang/xbotcon/build/CMakeFiles/XBOTCON.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/XBOTCON.dir/depend
