cmake_minimum_required(VERSION 3.3)

set(APP_NAME "KinematicChainPuma")
project(${APP_NAME})

set(INC_DIR include)
set(SRC_DIR src)
set(DEPS_DIR dependencies)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IFX_APP_BUILD_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

add_definitions(
        -DGLEW_STATIC
)

#---------------------------------
# MACROS
#---------------------------------

MACRO(FIND_INCLUDE_DIR return_list regex)
    FILE(GLOB_RECURSE new_list ${regex})
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        string(REGEX MATCH ^.*/include dir ${file_path})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

# INCLUDE AUTOMATIC SEARCH
FIND_INCLUDE_DIR(INC_DIRS ./include/*.h)
include_directories(${INC_DIR} ${INC_DIR})

# SOURCES AUTOMATIC SEARCH
file(GLOB_RECURSE SRC_FILES src/*.cpp)
set(SOURCE_FILES )

add_executable(${APP_NAME} ${SOURCE_FILES} ${SRC_FILES})


#----------------------------------
# EXTERNAL
#----------------------------------

add_subdirectory(${IFX_ROOT}/${DEPS_DIR} ${IFX_LIB_DEPS_BUILD_DIR})

include_directories(${IFX_ROOT}/${DEPS_DIR}/glfw-3.1.2/${INC_DIR})
include_directories(${IFX_ROOT}/${DEPS_DIR}/assimp-3.1.1/${INC_DIR})
include_directories(${IFX_ROOT}/${DEPS_DIR}/SOIL/src)
include_directories(${IFX_ROOT}/${DEPS_DIR}/glm/${INC_DIR})
include_directories(${IFX_ROOT}/${DEPS_DIR}/glew-2.0.0/include)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

#---------------------------------
# IFX LIBS
#---------------------------------

add_subdirectory(${IFX_ROOT} ${IFX_LIB_BUILD_DIR})

include_directories(${IFX_ROOT}/modules/controls/controls/${INC_DIR})
include_directories(${IFX_ROOT}/modules/graphics/${INC_DIR})
include_directories(${IFX_ROOT}/modules/object/object/${INC_DIR})
include_directories(${IFX_ROOT}/modules/common/resources/${INC_DIR})
include_directories(${IFX_ROOT}/modules/common/math/${INC_DIR})
include_directories(${IFX_ROOT}/modules/vr/${INC_DIR})
include_directories(${IFX_ROOT}/modules/game/game/${INC_DIR})
include_directories(${IFX_ROOT}/modules/physics/physics/${INC_DIR})
include_directories(${IFX_ROOT}/modules/gui/gui/${INC_DIR})
include_directories(${IFX_ROOT}/modules/gui/engine_gui/${INC_DIR})

#---------------------------------
# LINK
#---------------------------------

target_link_libraries(${APP_NAME} engine_gui_ifx
        game_ifx graphics_ifx physics_ifx
        resources_ifx controls_ifx object_ifx math_ifx
        vr_ifx gui_ifx)

target_link_libraries(${APP_NAME} SOIL)
target_link_libraries(${APP_NAME} assimp)
target_link_libraries(${APP_NAME} glfw ${GLFW_LIBRARIES})
target_link_libraries(${APP_NAME} ${OPENGL_LIBRARIES})
target_link_libraries(${APP_NAME} glew20)
