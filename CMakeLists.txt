cmake_minimum_required(VERSION 3.28)
project(progettopf VERSION 0.1.0)


include(CTest)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#Require C++17 for ROOT data analysis server
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


string(APPEND CMAKE_CXX_FLAGS
      " -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
      " -Wshadow -Wimplicit-fallthrough -Wextra-semi -Wold-style-cast"
      " -fno-omit-frame-pointer")


if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  string(APPEND CMAKE_CXX_FLAGS " -D_GLIBCXX_ASSERTIONS")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  string(APPEND CMAKE_CXX_FLAGS " -D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE")
endif()


string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address,undefined")
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  string(APPEND CMAKE_CXX_FLAGS " -D_GLIBCXX_SANITIZE_STD_ALLOCATOR")
endif()
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined")


find_package(SFML 2.6 COMPONENTS graphics REQUIRED)
find_package(ROOT REQUIRED COMPONENTS Hist Graf Gpad)


add_executable(boids
    main.cpp
    boids.cpp
    flock.cpp
    statistics.cpp
    sfml_rendering.cpp
    simulation.cpp
    graphs.cpp
    input_parameters.cpp)

target_link_libraries(boids PRIVATE sfml-graphics)
target_link_libraries(
    boids
    PRIVATE
    ROOT::Hist
    ROOT::Graf
    ROOT::Gpad
)

if (BUILD_TESTING)

  
add_executable(boids.t test.t.cpp boids.cpp flock.cpp statistics.cpp)
 
  add_test(NAME boids.t COMMAND boids.t)

endif()