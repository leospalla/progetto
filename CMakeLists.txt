cmake_minimum_required(VERSION 3.16)
project(boids VERSION 0.1.0)

# abilita il supporto per i test, tra cui l'opzione BUILD_TESTING usata sotto
include(CTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# richiedi l'uso di C++17, senza estensioni non-standard offerte dal compilatore usato 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# abilita warning
string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra")

# abilita l'address sanitizer e l'undefined-behaviour sanitizer in debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address,undefined -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined -fno-omit-frame-pointer")

# aggiunge l'eseguibile main nella directory build/
add_executable(main main.cpp boids.cpp flock.cpp vector.cpp)

# si attiva se è abilitato il testing
if (BUILD_TESTING)

# Aggiunge l'eseguibile vector.t per il test della classe Vector
add_executable(vector.t vector.test.cpp vector.cpp)
add_test(NAME vector.t COMMAND vector.t)
# Aggiunge l'eseguibile boids.t per il test della classe Boid
add_executable(boids.t boids.test.cpp boids.cpp vector.cpp)
add_test(NAME boids.t COMMAND boids.t)
# aggiunge l'eseguibile flock.t per il test della classe Flock
add_executable(flock.t flock.test.cpp flock.cpp boids.cpp vector.cpp)
add_test(NAME flock.t COMMAND flock.t)

endif() #finisce la parte di testing

#automatizza la formattazione con clang-format
find_program(CLANG_FORMAT_EXECUTABLE clang-format)
if (CLANG_FORMAT_EXECUTABLE)
    
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.hpp)
add_custom_target(format COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${ALL_SOURCE_FILES} COMMENT "Formatting code using clang-format" VERBATIM)

endif()