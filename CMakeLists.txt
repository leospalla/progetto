cmake_minimum_required(VERSION 3.16)
project(boids VERSION 0.1.0)

# Enable support for tests, including the BUILD_TESTING option used below
include(CTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Require the use of C++17, without non-standard compiler extensions
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Enable compiler warnings
string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra")

# Enable AddressSanitizer and UndefinedBehaviorSanitizer in debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address,undefined -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined -fno-omit-frame-pointer")

# Add the main executable in the build/ directory
add_executable(main main.cpp boid.cpp flock.cpp vector.cpp)

# Activate if testing is enabled
if (BUILD_TESTING)

# Add the vector.t executable for testing the Vector class
add_executable(vector.t vector.test.cpp vector.cpp)
add_test(NAME vector.t COMMAND vector.t)
# Add the boid.t executable for testing the Boid class
add_executable(boid.t boid.test.cpp boid.cpp vector.cpp)
add_test(NAME boid.t COMMAND boid.t)
# Add the flock.t executable for testing the Flock class
add_executable(flock.t flock.test.cpp flock.cpp boid.cpp vector.cpp)
add_test(NAME flock.t COMMAND flock.t)

endif() # End of testing section

# Automatize code formatting using clang-format
find_program(CLANG_FORMAT_EXECUTABLE clang-format)
if (CLANG_FORMAT_EXECUTABLE)
    
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.hpp)
add_custom_target(format COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${ALL_SOURCE_FILES} COMMENT "Formatting code using clang-format")

endif()
