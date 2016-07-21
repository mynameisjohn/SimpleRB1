cmake_minimum_required(VERSION 2.8)

project(SimpleRB1)

# SDL2
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Modules")
find_package(SDL2)
find_package(OpenGL)
find_package(GLEW)

# Python libraries for pyliaison
if (WIN32)
	add_definitions(-DGLEW_STATIC)
	set(PYTHON_LIBRARY C:/Python35/libs/python35_d.lib)
	set(PYTHON_INCLUDE_DIR C:/Python35/include)
	set(SDL2_LIBS ${SDL2_LIBRARY} ${SDLMAIN_LIBRARY})
else(WIN32)
	set(PYTHON_LIBRARY /usr/local/lib/libpython3.5m.a)
	set(PYTHON_INCLUDE_DIR /usr/local/include/python3.5m)
	set(SDL2_LIBS ${SDL2_LIBRARY})
endif(WIN32)

# Source files, include files, scripts
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
file(GLOB SHADERS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*)

# Create source groups
source_group("Scripts" FILES ${SCRIPTS})
source_group("Shaders" FILES ${SHADERS})
source_group("Source" FILES ${SOURCES})
source_group("Include" FILES ${HEADERS})

# Pyliaison, which has its own folder and source file]
file(GLOB PYL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/pyl/*.cpp)
file(GLOB PYL_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/pyl/*.h)
add_library(PyLiaison ${PYL_SOURCES} ${PYL_HEADERS})
target_include_directories(PyLiaison PUBLIC ${PYL_HEADERS} ${PYTHON_INCLUDE_DIR} C:/Libraries/glm)

# Add the SimpleRB1 executable, which depends on source, include, and scripts
add_executable(SimpleRB1 ${SOURCES} ${HEADERS} ${SCRIPTS})

# Make sure it gets its include paths
target_include_directories(SimpleRB1 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${PYTHON_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/pyl ${SDL2_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} C:/Libraries/glm)
target_link_libraries(SimpleRB1 LINK_PUBLIC PyLiaison ${PYTHON_LIBRARY} ${SDL2_LIBS} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
