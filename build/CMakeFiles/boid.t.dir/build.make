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
CMAKE_SOURCE_DIR = /home/leospallacci/boids/progetto

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leospallacci/boids/progetto/build

# Include any dependencies generated for this target.
include CMakeFiles/boid.t.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/boid.t.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/boid.t.dir/flags.make

CMakeFiles/boid.t.dir/boid.test.cpp.o: CMakeFiles/boid.t.dir/flags.make
CMakeFiles/boid.t.dir/boid.test.cpp.o: ../boid.test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/boid.t.dir/boid.test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/boid.t.dir/boid.test.cpp.o -c /home/leospallacci/boids/progetto/boid.test.cpp

CMakeFiles/boid.t.dir/boid.test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/boid.t.dir/boid.test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/boid.test.cpp > CMakeFiles/boid.t.dir/boid.test.cpp.i

CMakeFiles/boid.t.dir/boid.test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/boid.t.dir/boid.test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/boid.test.cpp -o CMakeFiles/boid.t.dir/boid.test.cpp.s

CMakeFiles/boid.t.dir/boid.cpp.o: CMakeFiles/boid.t.dir/flags.make
CMakeFiles/boid.t.dir/boid.cpp.o: ../boid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/boid.t.dir/boid.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/boid.t.dir/boid.cpp.o -c /home/leospallacci/boids/progetto/boid.cpp

CMakeFiles/boid.t.dir/boid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/boid.t.dir/boid.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/boid.cpp > CMakeFiles/boid.t.dir/boid.cpp.i

CMakeFiles/boid.t.dir/boid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/boid.t.dir/boid.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/boid.cpp -o CMakeFiles/boid.t.dir/boid.cpp.s

CMakeFiles/boid.t.dir/vector.cpp.o: CMakeFiles/boid.t.dir/flags.make
CMakeFiles/boid.t.dir/vector.cpp.o: ../vector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/boid.t.dir/vector.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/boid.t.dir/vector.cpp.o -c /home/leospallacci/boids/progetto/vector.cpp

CMakeFiles/boid.t.dir/vector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/boid.t.dir/vector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/vector.cpp > CMakeFiles/boid.t.dir/vector.cpp.i

CMakeFiles/boid.t.dir/vector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/boid.t.dir/vector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/vector.cpp -o CMakeFiles/boid.t.dir/vector.cpp.s

# Object files for target boid.t
boid_t_OBJECTS = \
"CMakeFiles/boid.t.dir/boid.test.cpp.o" \
"CMakeFiles/boid.t.dir/boid.cpp.o" \
"CMakeFiles/boid.t.dir/vector.cpp.o"

# External object files for target boid.t
boid_t_EXTERNAL_OBJECTS =

boid.t: CMakeFiles/boid.t.dir/boid.test.cpp.o
boid.t: CMakeFiles/boid.t.dir/boid.cpp.o
boid.t: CMakeFiles/boid.t.dir/vector.cpp.o
boid.t: CMakeFiles/boid.t.dir/build.make
boid.t: CMakeFiles/boid.t.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable boid.t"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/boid.t.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/boid.t.dir/build: boid.t

.PHONY : CMakeFiles/boid.t.dir/build

CMakeFiles/boid.t.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/boid.t.dir/cmake_clean.cmake
.PHONY : CMakeFiles/boid.t.dir/clean

CMakeFiles/boid.t.dir/depend:
	cd /home/leospallacci/boids/progetto/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leospallacci/boids/progetto /home/leospallacci/boids/progetto /home/leospallacci/boids/progetto/build /home/leospallacci/boids/progetto/build /home/leospallacci/boids/progetto/build/CMakeFiles/boid.t.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/boid.t.dir/depend

