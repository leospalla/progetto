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
include CMakeFiles/vector.t.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/vector.t.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/vector.t.dir/flags.make

CMakeFiles/vector.t.dir/vector.test.cpp.o: CMakeFiles/vector.t.dir/flags.make
CMakeFiles/vector.t.dir/vector.test.cpp.o: ../vector.test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/vector.t.dir/vector.test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vector.t.dir/vector.test.cpp.o -c /home/leospallacci/boids/progetto/vector.test.cpp

CMakeFiles/vector.t.dir/vector.test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vector.t.dir/vector.test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/vector.test.cpp > CMakeFiles/vector.t.dir/vector.test.cpp.i

CMakeFiles/vector.t.dir/vector.test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vector.t.dir/vector.test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/vector.test.cpp -o CMakeFiles/vector.t.dir/vector.test.cpp.s

CMakeFiles/vector.t.dir/vector.cpp.o: CMakeFiles/vector.t.dir/flags.make
CMakeFiles/vector.t.dir/vector.cpp.o: ../vector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/vector.t.dir/vector.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vector.t.dir/vector.cpp.o -c /home/leospallacci/boids/progetto/vector.cpp

CMakeFiles/vector.t.dir/vector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vector.t.dir/vector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/vector.cpp > CMakeFiles/vector.t.dir/vector.cpp.i

CMakeFiles/vector.t.dir/vector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vector.t.dir/vector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/vector.cpp -o CMakeFiles/vector.t.dir/vector.cpp.s

# Object files for target vector.t
vector_t_OBJECTS = \
"CMakeFiles/vector.t.dir/vector.test.cpp.o" \
"CMakeFiles/vector.t.dir/vector.cpp.o"

# External object files for target vector.t
vector_t_EXTERNAL_OBJECTS =

vector.t: CMakeFiles/vector.t.dir/vector.test.cpp.o
vector.t: CMakeFiles/vector.t.dir/vector.cpp.o
vector.t: CMakeFiles/vector.t.dir/build.make
vector.t: CMakeFiles/vector.t.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable vector.t"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vector.t.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/vector.t.dir/build: vector.t

.PHONY : CMakeFiles/vector.t.dir/build

CMakeFiles/vector.t.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/vector.t.dir/cmake_clean.cmake
.PHONY : CMakeFiles/vector.t.dir/clean

CMakeFiles/vector.t.dir/depend:
	cd /home/leospallacci/boids/progetto/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leospallacci/boids/progetto /home/leospallacci/boids/progetto /home/leospallacci/boids/progetto/build /home/leospallacci/boids/progetto/build /home/leospallacci/boids/progetto/build/CMakeFiles/vector.t.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/vector.t.dir/depend
