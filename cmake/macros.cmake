macro(target_rosify target)
    target_link_libraries(${target} PRIVATE ${catkin_LIBRARIES})
    target_include_directories(${target} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS} src/util)
    add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${MROVER_CPP_COMPILE_OPTIONS}>)

    # Installing is necessary for roslaunch to find the node
    install(TARGETS ${target}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endmacro()

macro(mrover_add_library name sources includes)
    file(GLOB_RECURSE CPP_LIB_SOURCES ${sources})
    add_library(${name} ${CPP_LIB_SOURCES})
    target_include_directories(${name} PUBLIC ${includes})
    target_rosify(${name})
endmacro()

macro(mrover_add_header_only_library name includes)
    add_library(${name} INTERFACE)
    target_include_directories(${name} INTERFACE ${includes})
endmacro()

macro(mrover_add_node name sources)
    file(GLOB_RECURSE NODE_SOURCES ${sources})
    add_executable(${name} ${NODE_SOURCES})
    target_rosify(${name})
endmacro()

macro(mrover_add_nodelet name sources includes)
    # A nodelet runs inside another process so it is a library
    mrover_add_library(${name}_nodelet ${sources} ${includes})
    # Also add a node for quick debugging
    mrover_add_node(${name} ${sources})
    # Explicitly tell CMake to re-build the nodelet when the node is built
    # CMake cannot tell these are dependent since a node dynamically (at runtime) loads the nodelet as a shared library
    add_dependencies(${name} ${name}_nodelet)
    # Allows the source code to split based on whether it is a node or a nodelet
    target_compile_definitions(${name}_nodelet PRIVATE MROVER_IS_NODELET)
    # Optional pre-compiled header (PCH) support
    if (ARGV3)
        target_precompile_headers(${name} PRIVATE ${ARGV3})
        target_precompile_headers(${name}_nodelet PRIVATE ${ARGV3})
    endif ()
endmacro()

macro(mrover_nodelet_link_libraries name)
    target_link_libraries(${name} PRIVATE ${ARGN})
    target_link_libraries(${name}_nodelet PRIVATE ${ARGN})
endmacro()

macro(mrover_nodelet_include_directories name)
    target_include_directories(${name} SYSTEM PRIVATE ${ARGN})
    target_include_directories(${name}_nodelet SYSTEM PRIVATE ${ARGN})
endmacro()

macro(mrover_nodelet_defines name)
    target_compile_definitions(${name} PRIVATE ${ARGN})
    target_compile_definitions(${name}_nodelet PRIVATE ${ARGN})
endmacro()

# Search a path for all files matching a glob pattern and extract the filenames
macro(extract_filenames directory_path out_file_names)
    file(GLOB_RECURSE full_paths ${directory_path})
    set(${out_file_names} "")
    foreach (FULL_PATH ${full_paths})
        get_filename_component(FILE_NAME ${FULL_PATH} NAME)
        list(APPEND ${out_file_names} ${FILE_NAME})
    endforeach ()
endmacro()
