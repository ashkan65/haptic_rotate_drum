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
CMAKE_SOURCE_DIR = /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/build

# Include any dependencies generated for this target.
include CMakeFiles/pest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pest.dir/flags.make

CMakeFiles/pest.dir/pest.cpp.o: CMakeFiles/pest.dir/flags.make
CMakeFiles/pest.dir/pest.cpp.o: ../pest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pest.dir/pest.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pest.dir/pest.cpp.o -c /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/pest.cpp

CMakeFiles/pest.dir/pest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pest.dir/pest.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/pest.cpp > CMakeFiles/pest.dir/pest.cpp.i

CMakeFiles/pest.dir/pest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pest.dir/pest.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/pest.cpp -o CMakeFiles/pest.dir/pest.cpp.s

CMakeFiles/pest.dir/pest.cpp.o.requires:

.PHONY : CMakeFiles/pest.dir/pest.cpp.o.requires

CMakeFiles/pest.dir/pest.cpp.o.provides: CMakeFiles/pest.dir/pest.cpp.o.requires
	$(MAKE) -f CMakeFiles/pest.dir/build.make CMakeFiles/pest.dir/pest.cpp.o.provides.build
.PHONY : CMakeFiles/pest.dir/pest.cpp.o.provides

CMakeFiles/pest.dir/pest.cpp.o.provides.build: CMakeFiles/pest.dir/pest.cpp.o


# Object files for target pest
pest_OBJECTS = \
"CMakeFiles/pest.dir/pest.cpp.o"

# External object files for target pest
pest_EXTERNAL_OBJECTS =

libpest.a: CMakeFiles/pest.dir/pest.cpp.o
libpest.a: CMakeFiles/pest.dir/build.make
libpest.a: CMakeFiles/pest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libpest.a"
	$(CMAKE_COMMAND) -P CMakeFiles/pest.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pest.dir/build: libpest.a

.PHONY : CMakeFiles/pest.dir/build

CMakeFiles/pest.dir/requires: CMakeFiles/pest.dir/pest.cpp.o.requires

.PHONY : CMakeFiles/pest.dir/requires

CMakeFiles/pest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pest.dir/clean

CMakeFiles/pest.dir/depend:
	cd /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/build /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/build /home/naghmehz/Downloads/Sensoray_CHAI3D-zhianli/build/CMakeFiles/pest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pest.dir/depend
