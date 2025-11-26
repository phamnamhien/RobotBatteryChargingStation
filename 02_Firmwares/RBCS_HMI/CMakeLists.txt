# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Chạy setup_ui.py chỉ khi CMakeLists.txt.squareline mới hơn
set(UI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/components/ui")
set(SQUARELINE_CMAKE "${UI_DIR}/CMakeLists.txt.squareline")
set(OUTPUT_CMAKE "${UI_DIR}/CMakeLists.txt")
set(SETUP_SCRIPT "${UI_DIR}/setup_ui.py")

if(EXISTS ${SQUARELINE_CMAKE} AND EXISTS ${SETUP_SCRIPT})
    # Kiểm tra timestamp
    if(${SQUARELINE_CMAKE} IS_NEWER_THAN ${OUTPUT_CMAKE})
        message(STATUS "Squareline CMakeLists updated, regenerating...")
        find_package(Python3 COMPONENTS Interpreter)
        execute_process(
            COMMAND ${Python3_EXECUTABLE} ${SETUP_SCRIPT}
            RESULT_VARIABLE SETUP_RESULT
        )
        if(SETUP_RESULT EQUAL 0)
            message(STATUS "✓ UI CMakeLists.txt regenerated")
        endif()
    endif()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(RBCS_HMI)