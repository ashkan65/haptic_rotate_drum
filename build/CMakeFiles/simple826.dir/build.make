# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/build

# Include any dependencies generated for this target.
include CMakeFiles/simple826.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/simple826.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simple826.dir/flags.make

CMakeFiles/simple826.dir/simple826.cpp.o: CMakeFiles/simple826.dir/flags.make
CMakeFiles/simple826.dir/simple826.cpp.o: ../simple826.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/simple826.dir/simple826.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simple826.dir/simple826.cpp.o -c /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/simple826.cpp

CMakeFiles/simple826.dir/simple826.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simple826.dir/simple826.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/simple826.cpp > CMakeFiles/simple826.dir/simple826.cpp.i

CMakeFiles/simple826.dir/simple826.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simple826.dir/simple826.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/simple826.cpp -o CMakeFiles/simple826.dir/simple826.cpp.s

CMakeFiles/simple826.dir/simple826.cpp.o.requires:

.PHONY : CMakeFiles/simple826.dir/simple826.cpp.o.requires

CMakeFiles/simple826.dir/simple826.cpp.o.provides: CMakeFiles/simple826.dir/simple826.cpp.o.requires
	$(MAKE) -f CMakeFiles/simple826.dir/build.make CMakeFiles/simple826.dir/simple826.cpp.o.provides.build
.PHONY : CMakeFiles/simple826.dir/simple826.cpp.o.provides

CMakeFiles/simple826.dir/simple826.cpp.o.provides.build: CMakeFiles/simple826.dir/simple826.cpp.o


# Object files for target simple826
simple826_OBJECTS = \
"CMakeFiles/simple826.dir/simple826.cpp.o"

# External object files for target simple826
simple826_EXTERNAL_OBJECTS =

libsimple826.a: CMakeFiles/simple826.dir/simple826.cpp.o
libsimple826.a: CMakeFiles/simple826.dir/build.make
libsimple826.a: CMakeFiles/simple826.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libsimple826.a"
	$(CMAKE_COMMAND) -P CMakeFiles/simple826.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple826.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simple826.dir/build: libsimple826.a

.PHONY : CMakeFiles/simple826.dir/build

CMakeFiles/simple826.dir/requires: CMakeFiles/simple826.dir/simple826.cpp.o.requires

.PHONY : CMakeFiles/simple826.dir/requires

CMakeFiles/simple826.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simple826.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simple826.dir/clean

CMakeFiles/simple826.dir/depend:
	cd /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2 /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2 /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/build /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/build /home/naghmehz/Desktop/Sensoray_CHAI3D-experiment2/build/CMakeFiles/simple826.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simple826.dir/depend

