cmake_minimum_required(VERSION 3.3)

SET(IFX_ROOT "${CMAKE_SOURCE_DIR}/InfinityXLib/")

SET(BUILD_DIR "./build/")

SET(IFX_APP_BUILD_DIR "${BUILD_DIR}/bin")
SET(IFX_LIB_BUILD_DIR "${BUILD_DIR}/InfinityX")
SET(IFX_LIB_DEPS_BUILD_DIR "${BUILD_DIR}/InfinityX/dependencies")

SET(IFX_UNITTESTS_BIN_BUILD_DIR "${BUILD_DIR}/InfinityX/unit_tests/bin")
SET(IFX_UNITTESTS_LIBS_BUILD_DIR "${BUILD_DIR}/InfinityX/unit_tests/libs")

SET(IFX_BUILD_UNITTESTS ON)

add_subdirectory(KinematicChainPuma)
