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
include CMakeFiles/flock.t.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/flock.t.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/flock.t.dir/flags.make

CMakeFiles/flock.t.dir/flock.test.cpp.o: CMakeFiles/flock.t.dir/flags.make
CMakeFiles/flock.t.dir/flock.test.cpp.o: ../flock.test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/flock.t.dir/flock.test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/flock.t.dir/flock.test.cpp.o -c /home/leospallacci/boids/progetto/flock.test.cpp

CMakeFiles/flock.t.dir/flock.test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flock.t.dir/flock.test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/flock.test.cpp > CMakeFiles/flock.t.dir/flock.test.cpp.i

CMakeFiles/flock.t.dir/flock.test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flock.t.dir/flock.test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/flock.test.cpp -o CMakeFiles/flock.t.dir/flock.test.cpp.s

CMakeFiles/flock.t.dir/flock.cpp.o: CMakeFiles/flock.t.dir/flags.make
CMakeFiles/flock.t.dir/flock.cpp.o: ../flock.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/flock.t.dir/flock.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/flock.t.dir/flock.cpp.o -c /home/leospallacci/boids/progetto/flock.cpp

CMakeFiles/flock.t.dir/flock.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flock.t.dir/flock.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/flock.cpp > CMakeFiles/flock.t.dir/flock.cpp.i

CMakeFiles/flock.t.dir/flock.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flock.t.dir/flock.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/flock.cpp -o CMakeFiles/flock.t.dir/flock.cpp.s

CMakeFiles/flock.t.dir/boid.cpp.o: CMakeFiles/flock.t.dir/flags.make
CMakeFiles/flock.t.dir/boid.cpp.o: ../boid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/flock.t.dir/boid.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/flock.t.dir/boid.cpp.o -c /home/leospallacci/boids/progetto/boid.cpp

CMakeFiles/flock.t.dir/boid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flock.t.dir/boid.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/boid.cpp > CMakeFiles/flock.t.dir/boid.cpp.i

CMakeFiles/flock.t.dir/boid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flock.t.dir/boid.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/boid.cpp -o CMakeFiles/flock.t.dir/boid.cpp.s

CMakeFiles/flock.t.dir/vector.cpp.o: CMakeFiles/flock.t.dir/flags.make
CMakeFiles/flock.t.dir/vector.cpp.o: ../vector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/flock.t.dir/vector.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/flock.t.dir/vector.cpp.o -c /home/leospallacci/boids/progetto/vector.cpp

CMakeFiles/flock.t.dir/vector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/flock.t.dir/vector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/vector.cpp > CMakeFiles/flock.t.dir/vector.cpp.i

CMakeFiles/flock.t.dir/vector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/flock.t.dir/vector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/vector.cpp -o CMakeFiles/flock.t.dir/vector.cpp.s

# Object files for target flock.t
flock_t_OBJECTS = \
"CMakeFiles/flock.t.dir/flock.test.cpp.o" \
"CMakeFiles/flock.t.dir/flock.cpp.o" \
"CMakeFiles/flock.t.dir/boid.cpp.o" \
"CMakeFiles/flock.t.dir/vector.cpp.o"

# External object files for target flock.t
flock_t_EXTERNAL_OBJECTS =

flock.t: CMakeFiles/flock.t.dir/flock.test.cpp.o
flock.t: CMakeFiles/flock.t.dir/flock.cpp.o
flock.t: CMakeFiles/flock.t.dir/boid.cpp.o
flock.t: CMakeFiles/flock.t.dir/vector.cpp.o
flock.t: CMakeFiles/flock.t.dir/build.make
flock.t: CMakeFiles/flock.t.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable flock.t"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/flock.t.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/flock.t.dir/build: flock.t

.PHONY : CMakeFiles/flock.t.dir/build

CMakeFiles/flock.t.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/flock.t.dir/cmake_clean.cmake
.PHONY : CMakeFiles/flock.t.dir/clean

CMakeFiles/flock.t.dir/depend:
	cd /home/leospallacci/boids/progetto/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leospallacci/boids/progetto /home/leospallacci/boids/progetto /home/leospallacci/boids/progetto/build /home/leospallacci/boids/progetto/build /home/leospallacci/boids/progetto/build/CMakeFiles/flock.t.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/flock.t.dir/depend

