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
include CMakeFiles/vector.test.cpp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/vector.test.cpp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/vector.test.cpp.dir/flags.make

CMakeFiles/vector.test.cpp.dir/vector.cpp.o: CMakeFiles/vector.test.cpp.dir/flags.make
CMakeFiles/vector.test.cpp.dir/vector.cpp.o: ../vector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/vector.test.cpp.dir/vector.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vector.test.cpp.dir/vector.cpp.o -c /home/leospallacci/boids/progetto/vector.cpp

CMakeFiles/vector.test.cpp.dir/vector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vector.test.cpp.dir/vector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/vector.cpp > CMakeFiles/vector.test.cpp.dir/vector.cpp.i

CMakeFiles/vector.test.cpp.dir/vector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vector.test.cpp.dir/vector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/vector.cpp -o CMakeFiles/vector.test.cpp.dir/vector.cpp.s

CMakeFiles/vector.test.cpp.dir/boids.test.cpp.o: CMakeFiles/vector.test.cpp.dir/flags.make
CMakeFiles/vector.test.cpp.dir/boids.test.cpp.o: ../boids.test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/vector.test.cpp.dir/boids.test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vector.test.cpp.dir/boids.test.cpp.o -c /home/leospallacci/boids/progetto/boids.test.cpp

CMakeFiles/vector.test.cpp.dir/boids.test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vector.test.cpp.dir/boids.test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/boids.test.cpp > CMakeFiles/vector.test.cpp.dir/boids.test.cpp.i

CMakeFiles/vector.test.cpp.dir/boids.test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vector.test.cpp.dir/boids.test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/boids.test.cpp -o CMakeFiles/vector.test.cpp.dir/boids.test.cpp.s

CMakeFiles/vector.test.cpp.dir/boids.cpp.o: CMakeFiles/vector.test.cpp.dir/flags.make
CMakeFiles/vector.test.cpp.dir/boids.cpp.o: ../boids.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/vector.test.cpp.dir/boids.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vector.test.cpp.dir/boids.cpp.o -c /home/leospallacci/boids/progetto/boids.cpp

CMakeFiles/vector.test.cpp.dir/boids.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vector.test.cpp.dir/boids.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leospallacci/boids/progetto/boids.cpp > CMakeFiles/vector.test.cpp.dir/boids.cpp.i

CMakeFiles/vector.test.cpp.dir/boids.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vector.test.cpp.dir/boids.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leospallacci/boids/progetto/boids.cpp -o CMakeFiles/vector.test.cpp.dir/boids.cpp.s

# Object files for target vector.test.cpp
vector_test_cpp_OBJECTS = \
"CMakeFiles/vector.test.cpp.dir/vector.cpp.o" \
"CMakeFiles/vector.test.cpp.dir/boids.test.cpp.o" \
"CMakeFiles/vector.test.cpp.dir/boids.cpp.o"

# External object files for target vector.test.cpp
vector_test_cpp_EXTERNAL_OBJECTS =

vector.test.cpp: CMakeFiles/vector.test.cpp.dir/vector.cpp.o
vector.test.cpp: CMakeFiles/vector.test.cpp.dir/boids.test.cpp.o
vector.test.cpp: CMakeFiles/vector.test.cpp.dir/boids.cpp.o
vector.test.cpp: CMakeFiles/vector.test.cpp.dir/build.make
vector.test.cpp: CMakeFiles/vector.test.cpp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leospallacci/boids/progetto/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable vector.test.cpp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vector.test.cpp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/vector.test.cpp.dir/build: vector.test.cpp

.PHONY : CMakeFiles/vector.test.cpp.dir/build

CMakeFiles/vector.test.cpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/vector.test.cpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/vector.test.cpp.dir/clean

CMakeFiles/vector.test.cpp.dir/depend:
	cd /home/leospallacci/boids/progetto/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leospallacci/boids/progetto /home/leospallacci/boids/progetto /home/leospallacci/boids/progetto/build /home/leospallacci/boids/progetto/build /home/leospallacci/boids/progetto/build/CMakeFiles/vector.test.cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/vector.test.cpp.dir/depend
