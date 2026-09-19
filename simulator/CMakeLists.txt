cmake_minimum_required(VERSION 3.10)
project(raytracer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(app main.cpp)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(png REQUIRED)
find_package(glfw3 CONFIG QUIET)
if(NOT TARGET glfw AND NOT TARGET glfw3)
  find_package(glfw3 REQUIRED)
endif()

if(PNG_FOUND)
    target_include_directories(app PRIVATE ${PNG_INCLUDE_DIRS})
    target_link_libraries(app PRIVATE ${PNG_LIBRARIES})
endif()

target_link_libraries(app PRIVATE OpenGL::GL GLEW::GLEW)
if(TARGET glfw)
  target_link_libraries(app PRIVATE glfw)
elseif(TARGET glfw3)
  target_link_libraries(app PRIVATE glfw3)
else()
  target_link_libraries(app PRIVATE glfw)
endif()

target_link_libraries(app 
    PRIVATE        GLUT::GLUT
)

if(WIN32)
  target_compile_definitions(raytracer PRIVATE GLEW_STATIC)
endif()
