# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src

# Include any dependencies generated for this target.
include CMakeFiles/test_ASIFT.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_ASIFT.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_ASIFT.dir/flags.make

CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o: test_ASIFT.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/test_ASIFT.cpp

CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/test_ASIFT.cpp > CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.i

CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/test_ASIFT.cpp -o CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.s

CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o.requires

CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o.provides: CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o.provides

CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o

CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o: ASIFT_matcher.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/ASIFT_matcher.cpp

CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/ASIFT_matcher.cpp > CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.i

CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/ASIFT_matcher.cpp -o CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.s

CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o.requires

CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o.provides: CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o.provides

CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o

CMakeFiles/test_ASIFT.dir/numerics1.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/numerics1.cpp.o: numerics1.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/numerics1.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/numerics1.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/numerics1.cpp

CMakeFiles/test_ASIFT.dir/numerics1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/numerics1.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/numerics1.cpp > CMakeFiles/test_ASIFT.dir/numerics1.cpp.i

CMakeFiles/test_ASIFT.dir/numerics1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/numerics1.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/numerics1.cpp -o CMakeFiles/test_ASIFT.dir/numerics1.cpp.s

CMakeFiles/test_ASIFT.dir/numerics1.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/numerics1.cpp.o.requires

CMakeFiles/test_ASIFT.dir/numerics1.cpp.o.provides: CMakeFiles/test_ASIFT.dir/numerics1.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/numerics1.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/numerics1.cpp.o.provides

CMakeFiles/test_ASIFT.dir/numerics1.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/numerics1.cpp.o

CMakeFiles/test_ASIFT.dir/frot.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/frot.cpp.o: frot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/frot.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/frot.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/frot.cpp

CMakeFiles/test_ASIFT.dir/frot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/frot.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/frot.cpp > CMakeFiles/test_ASIFT.dir/frot.cpp.i

CMakeFiles/test_ASIFT.dir/frot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/frot.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/frot.cpp -o CMakeFiles/test_ASIFT.dir/frot.cpp.s

CMakeFiles/test_ASIFT.dir/frot.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/frot.cpp.o.requires

CMakeFiles/test_ASIFT.dir/frot.cpp.o.provides: CMakeFiles/test_ASIFT.dir/frot.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/frot.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/frot.cpp.o.provides

CMakeFiles/test_ASIFT.dir/frot.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/frot.cpp.o

CMakeFiles/test_ASIFT.dir/splines.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/splines.cpp.o: splines.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/splines.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/splines.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/splines.cpp

CMakeFiles/test_ASIFT.dir/splines.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/splines.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/splines.cpp > CMakeFiles/test_ASIFT.dir/splines.cpp.i

CMakeFiles/test_ASIFT.dir/splines.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/splines.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/splines.cpp -o CMakeFiles/test_ASIFT.dir/splines.cpp.s

CMakeFiles/test_ASIFT.dir/splines.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/splines.cpp.o.requires

CMakeFiles/test_ASIFT.dir/splines.cpp.o.provides: CMakeFiles/test_ASIFT.dir/splines.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/splines.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/splines.cpp.o.provides

CMakeFiles/test_ASIFT.dir/splines.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/splines.cpp.o

CMakeFiles/test_ASIFT.dir/fproj.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/fproj.cpp.o: fproj.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/fproj.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/fproj.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/fproj.cpp

CMakeFiles/test_ASIFT.dir/fproj.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/fproj.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/fproj.cpp > CMakeFiles/test_ASIFT.dir/fproj.cpp.i

CMakeFiles/test_ASIFT.dir/fproj.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/fproj.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/fproj.cpp -o CMakeFiles/test_ASIFT.dir/fproj.cpp.s

CMakeFiles/test_ASIFT.dir/fproj.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/fproj.cpp.o.requires

CMakeFiles/test_ASIFT.dir/fproj.cpp.o.provides: CMakeFiles/test_ASIFT.dir/fproj.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/fproj.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/fproj.cpp.o.provides

CMakeFiles/test_ASIFT.dir/fproj.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/fproj.cpp.o

CMakeFiles/test_ASIFT.dir/library.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/library.cpp.o: library.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/library.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/library.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/library.cpp

CMakeFiles/test_ASIFT.dir/library.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/library.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/library.cpp > CMakeFiles/test_ASIFT.dir/library.cpp.i

CMakeFiles/test_ASIFT.dir/library.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/library.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/library.cpp -o CMakeFiles/test_ASIFT.dir/library.cpp.s

CMakeFiles/test_ASIFT.dir/library.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/library.cpp.o.requires

CMakeFiles/test_ASIFT.dir/library.cpp.o.provides: CMakeFiles/test_ASIFT.dir/library.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/library.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/library.cpp.o.provides

CMakeFiles/test_ASIFT.dir/library.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/library.cpp.o

CMakeFiles/test_ASIFT.dir/flimage.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/flimage.cpp.o: flimage.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/flimage.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/flimage.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/flimage.cpp

CMakeFiles/test_ASIFT.dir/flimage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/flimage.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/flimage.cpp > CMakeFiles/test_ASIFT.dir/flimage.cpp.i

CMakeFiles/test_ASIFT.dir/flimage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/flimage.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/flimage.cpp -o CMakeFiles/test_ASIFT.dir/flimage.cpp.s

CMakeFiles/test_ASIFT.dir/flimage.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/flimage.cpp.o.requires

CMakeFiles/test_ASIFT.dir/flimage.cpp.o.provides: CMakeFiles/test_ASIFT.dir/flimage.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/flimage.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/flimage.cpp.o.provides

CMakeFiles/test_ASIFT.dir/flimage.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/flimage.cpp.o

CMakeFiles/test_ASIFT.dir/filter.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/filter.cpp.o: filter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/filter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/filter.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/filter.cpp

CMakeFiles/test_ASIFT.dir/filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/filter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/filter.cpp > CMakeFiles/test_ASIFT.dir/filter.cpp.i

CMakeFiles/test_ASIFT.dir/filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/filter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/filter.cpp -o CMakeFiles/test_ASIFT.dir/filter.cpp.s

CMakeFiles/test_ASIFT.dir/filter.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/filter.cpp.o.requires

CMakeFiles/test_ASIFT.dir/filter.cpp.o.provides: CMakeFiles/test_ASIFT.dir/filter.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/filter.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/filter.cpp.o.provides

CMakeFiles/test_ASIFT.dir/filter.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/filter.cpp.o

CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o: demo_lib_sift.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/demo_lib_sift.cpp

CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/demo_lib_sift.cpp > CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.i

CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/demo_lib_sift.cpp -o CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.s

CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o.requires

CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o.provides: CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o.provides

CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o

CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o: compute_asift_keypoints.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/compute_asift_keypoints.cpp

CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/compute_asift_keypoints.cpp > CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.i

CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/compute_asift_keypoints.cpp -o CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.s

CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o.requires

CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o.provides: CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o.provides

CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o

CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o: compute_asift_matches.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_12)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/compute_asift_matches.cpp

CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/compute_asift_matches.cpp > CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.i

CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/compute_asift_matches.cpp -o CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.s

CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o.requires

CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o.provides: CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o.provides

CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o

CMakeFiles/test_ASIFT.dir/orsa.cpp.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/orsa.cpp.o: orsa.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_13)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_ASIFT.dir/orsa.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_ASIFT.dir/orsa.cpp.o -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/orsa.cpp

CMakeFiles/test_ASIFT.dir/orsa.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_ASIFT.dir/orsa.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/orsa.cpp > CMakeFiles/test_ASIFT.dir/orsa.cpp.i

CMakeFiles/test_ASIFT.dir/orsa.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_ASIFT.dir/orsa.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/orsa.cpp -o CMakeFiles/test_ASIFT.dir/orsa.cpp.s

CMakeFiles/test_ASIFT.dir/orsa.cpp.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/orsa.cpp.o.requires

CMakeFiles/test_ASIFT.dir/orsa.cpp.o.provides: CMakeFiles/test_ASIFT.dir/orsa.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/orsa.cpp.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/orsa.cpp.o.provides

CMakeFiles/test_ASIFT.dir/orsa.cpp.o.provides.build: CMakeFiles/test_ASIFT.dir/orsa.cpp.o

CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o: CMakeFiles/test_ASIFT.dir/flags.make
CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o: io_png/io_png.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles $(CMAKE_PROGRESS_14)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o   -c /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/io_png/io_png.c

CMakeFiles/test_ASIFT.dir/io_png/io_png.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test_ASIFT.dir/io_png/io_png.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/io_png/io_png.c > CMakeFiles/test_ASIFT.dir/io_png/io_png.c.i

CMakeFiles/test_ASIFT.dir/io_png/io_png.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test_ASIFT.dir/io_png/io_png.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/io_png/io_png.c -o CMakeFiles/test_ASIFT.dir/io_png/io_png.c.s

CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o.requires:
.PHONY : CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o.requires

CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o.provides: CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o.requires
	$(MAKE) -f CMakeFiles/test_ASIFT.dir/build.make CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o.provides.build
.PHONY : CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o.provides

CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o.provides.build: CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o

# Object files for target test_ASIFT
test_ASIFT_OBJECTS = \
"CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o" \
"CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o" \
"CMakeFiles/test_ASIFT.dir/numerics1.cpp.o" \
"CMakeFiles/test_ASIFT.dir/frot.cpp.o" \
"CMakeFiles/test_ASIFT.dir/splines.cpp.o" \
"CMakeFiles/test_ASIFT.dir/fproj.cpp.o" \
"CMakeFiles/test_ASIFT.dir/library.cpp.o" \
"CMakeFiles/test_ASIFT.dir/flimage.cpp.o" \
"CMakeFiles/test_ASIFT.dir/filter.cpp.o" \
"CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o" \
"CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o" \
"CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o" \
"CMakeFiles/test_ASIFT.dir/orsa.cpp.o" \
"CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o"

# External object files for target test_ASIFT
test_ASIFT_EXTERNAL_OBJECTS =

test_ASIFT: CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/numerics1.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/frot.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/splines.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/fproj.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/library.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/flimage.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/filter.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/orsa.cpp.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o
test_ASIFT: CMakeFiles/test_ASIFT.dir/build.make
test_ASIFT: io_png/libs/png/libpng.a
test_ASIFT: io_png/libs/zlib/libzlib.a
test_ASIFT: libMatch/libMatch.a
test_ASIFT: libNumerics/libNumerics.a
test_ASIFT: CMakeFiles/test_ASIFT.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable test_ASIFT"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_ASIFT.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_ASIFT.dir/build: test_ASIFT
.PHONY : CMakeFiles/test_ASIFT.dir/build

CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/test_ASIFT.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/ASIFT_matcher.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/numerics1.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/frot.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/splines.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/fproj.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/library.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/flimage.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/filter.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/demo_lib_sift.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/compute_asift_keypoints.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/compute_asift_matches.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/orsa.cpp.o.requires
CMakeFiles/test_ASIFT.dir/requires: CMakeFiles/test_ASIFT.dir/io_png/io_png.c.o.requires
.PHONY : CMakeFiles/test_ASIFT.dir/requires

CMakeFiles/test_ASIFT.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_ASIFT.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_ASIFT.dir/clean

CMakeFiles/test_ASIFT.dir/depend:
	cd /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src /home/harle/catkin_ws/src/BaxterInterface/ASIFT_tests/demo_ASIFT_src/CMakeFiles/test_ASIFT.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_ASIFT.dir/depend
