# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lik/rm_power_rune

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lik/rm_power_rune/build

# Include any dependencies generated for this target.
include CMakeFiles/power_rune.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/power_rune.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/power_rune.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/power_rune.dir/flags.make

CMakeFiles/power_rune.dir/main.cpp.o: CMakeFiles/power_rune.dir/flags.make
CMakeFiles/power_rune.dir/main.cpp.o: /home/lik/rm_power_rune/main.cpp
CMakeFiles/power_rune.dir/main.cpp.o: CMakeFiles/power_rune.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lik/rm_power_rune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/power_rune.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/power_rune.dir/main.cpp.o -MF CMakeFiles/power_rune.dir/main.cpp.o.d -o CMakeFiles/power_rune.dir/main.cpp.o -c /home/lik/rm_power_rune/main.cpp

CMakeFiles/power_rune.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/power_rune.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lik/rm_power_rune/main.cpp > CMakeFiles/power_rune.dir/main.cpp.i

CMakeFiles/power_rune.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/power_rune.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lik/rm_power_rune/main.cpp -o CMakeFiles/power_rune.dir/main.cpp.s

CMakeFiles/power_rune.dir/src/Calculate.cpp.o: CMakeFiles/power_rune.dir/flags.make
CMakeFiles/power_rune.dir/src/Calculate.cpp.o: /home/lik/rm_power_rune/src/Calculate.cpp
CMakeFiles/power_rune.dir/src/Calculate.cpp.o: CMakeFiles/power_rune.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lik/rm_power_rune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/power_rune.dir/src/Calculate.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/power_rune.dir/src/Calculate.cpp.o -MF CMakeFiles/power_rune.dir/src/Calculate.cpp.o.d -o CMakeFiles/power_rune.dir/src/Calculate.cpp.o -c /home/lik/rm_power_rune/src/Calculate.cpp

CMakeFiles/power_rune.dir/src/Calculate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/power_rune.dir/src/Calculate.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lik/rm_power_rune/src/Calculate.cpp > CMakeFiles/power_rune.dir/src/Calculate.cpp.i

CMakeFiles/power_rune.dir/src/Calculate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/power_rune.dir/src/Calculate.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lik/rm_power_rune/src/Calculate.cpp -o CMakeFiles/power_rune.dir/src/Calculate.cpp.s

CMakeFiles/power_rune.dir/src/Detect.cpp.o: CMakeFiles/power_rune.dir/flags.make
CMakeFiles/power_rune.dir/src/Detect.cpp.o: /home/lik/rm_power_rune/src/Detect.cpp
CMakeFiles/power_rune.dir/src/Detect.cpp.o: CMakeFiles/power_rune.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lik/rm_power_rune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/power_rune.dir/src/Detect.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/power_rune.dir/src/Detect.cpp.o -MF CMakeFiles/power_rune.dir/src/Detect.cpp.o.d -o CMakeFiles/power_rune.dir/src/Detect.cpp.o -c /home/lik/rm_power_rune/src/Detect.cpp

CMakeFiles/power_rune.dir/src/Detect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/power_rune.dir/src/Detect.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lik/rm_power_rune/src/Detect.cpp > CMakeFiles/power_rune.dir/src/Detect.cpp.i

CMakeFiles/power_rune.dir/src/Detect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/power_rune.dir/src/Detect.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lik/rm_power_rune/src/Detect.cpp -o CMakeFiles/power_rune.dir/src/Detect.cpp.s

CMakeFiles/power_rune.dir/src/Param.cpp.o: CMakeFiles/power_rune.dir/flags.make
CMakeFiles/power_rune.dir/src/Param.cpp.o: /home/lik/rm_power_rune/src/Param.cpp
CMakeFiles/power_rune.dir/src/Param.cpp.o: CMakeFiles/power_rune.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lik/rm_power_rune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/power_rune.dir/src/Param.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/power_rune.dir/src/Param.cpp.o -MF CMakeFiles/power_rune.dir/src/Param.cpp.o.d -o CMakeFiles/power_rune.dir/src/Param.cpp.o -c /home/lik/rm_power_rune/src/Param.cpp

CMakeFiles/power_rune.dir/src/Param.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/power_rune.dir/src/Param.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lik/rm_power_rune/src/Param.cpp > CMakeFiles/power_rune.dir/src/Param.cpp.i

CMakeFiles/power_rune.dir/src/Param.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/power_rune.dir/src/Param.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lik/rm_power_rune/src/Param.cpp -o CMakeFiles/power_rune.dir/src/Param.cpp.s

CMakeFiles/power_rune.dir/src/PowerRune.cpp.o: CMakeFiles/power_rune.dir/flags.make
CMakeFiles/power_rune.dir/src/PowerRune.cpp.o: /home/lik/rm_power_rune/src/PowerRune.cpp
CMakeFiles/power_rune.dir/src/PowerRune.cpp.o: CMakeFiles/power_rune.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lik/rm_power_rune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/power_rune.dir/src/PowerRune.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/power_rune.dir/src/PowerRune.cpp.o -MF CMakeFiles/power_rune.dir/src/PowerRune.cpp.o.d -o CMakeFiles/power_rune.dir/src/PowerRune.cpp.o -c /home/lik/rm_power_rune/src/PowerRune.cpp

CMakeFiles/power_rune.dir/src/PowerRune.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/power_rune.dir/src/PowerRune.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lik/rm_power_rune/src/PowerRune.cpp > CMakeFiles/power_rune.dir/src/PowerRune.cpp.i

CMakeFiles/power_rune.dir/src/PowerRune.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/power_rune.dir/src/PowerRune.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lik/rm_power_rune/src/PowerRune.cpp -o CMakeFiles/power_rune.dir/src/PowerRune.cpp.s

# Object files for target power_rune
power_rune_OBJECTS = \
"CMakeFiles/power_rune.dir/main.cpp.o" \
"CMakeFiles/power_rune.dir/src/Calculate.cpp.o" \
"CMakeFiles/power_rune.dir/src/Detect.cpp.o" \
"CMakeFiles/power_rune.dir/src/Param.cpp.o" \
"CMakeFiles/power_rune.dir/src/PowerRune.cpp.o"

# External object files for target power_rune
power_rune_EXTERNAL_OBJECTS =

/home/lik/rm_power_rune/bin/power_rune: CMakeFiles/power_rune.dir/main.cpp.o
/home/lik/rm_power_rune/bin/power_rune: CMakeFiles/power_rune.dir/src/Calculate.cpp.o
/home/lik/rm_power_rune/bin/power_rune: CMakeFiles/power_rune.dir/src/Detect.cpp.o
/home/lik/rm_power_rune/bin/power_rune: CMakeFiles/power_rune.dir/src/Param.cpp.o
/home/lik/rm_power_rune/bin/power_rune: CMakeFiles/power_rune.dir/src/PowerRune.cpp.o
/home/lik/rm_power_rune/bin/power_rune: CMakeFiles/power_rune.dir/build.make
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_gapi.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_stitching.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_aruco.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_barcode.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_bgsegm.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_bioinspired.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_ccalib.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_dnn_objdetect.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_dnn_superres.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_dpm.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_face.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_freetype.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_fuzzy.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_hfs.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_img_hash.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_intensity_transform.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_line_descriptor.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_mcc.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_quality.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_rapid.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_reg.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_rgbd.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_saliency.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_stereo.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_structured_light.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_superres.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_surface_matching.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_tracking.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_videostab.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_wechat_qrcode.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_xfeatures2d.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_xobjdetect.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_xphoto.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/lib/libceres.so.2.0.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_shape.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_highgui.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_datasets.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_plot.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_text.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_ml.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_phase_unwrapping.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_optflow.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_ximgproc.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_video.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_videoio.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_imgcodecs.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_objdetect.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_calib3d.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_dnn.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_features2d.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_flann.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_photo.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_imgproc.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/local/lib/libopencv_core.so.4.6.0
/home/lik/rm_power_rune/bin/power_rune: /usr/lib/x86_64-linux-gnu/libglog.so.0.4.0
/home/lik/rm_power_rune/bin/power_rune: /usr/lib/x86_64-linux-gnu/libunwind.so
/home/lik/rm_power_rune/bin/power_rune: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
/home/lik/rm_power_rune/bin/power_rune: CMakeFiles/power_rune.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lik/rm_power_rune/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable /home/lik/rm_power_rune/bin/power_rune"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/power_rune.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/power_rune.dir/build: /home/lik/rm_power_rune/bin/power_rune
.PHONY : CMakeFiles/power_rune.dir/build

CMakeFiles/power_rune.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/power_rune.dir/cmake_clean.cmake
.PHONY : CMakeFiles/power_rune.dir/clean

CMakeFiles/power_rune.dir/depend:
	cd /home/lik/rm_power_rune/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lik/rm_power_rune /home/lik/rm_power_rune /home/lik/rm_power_rune/build /home/lik/rm_power_rune/build /home/lik/rm_power_rune/build/CMakeFiles/power_rune.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/power_rune.dir/depend

