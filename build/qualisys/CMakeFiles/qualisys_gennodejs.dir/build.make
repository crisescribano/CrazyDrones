# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/cristinaescribano/CrazyDrones/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cristinaescribano/CrazyDrones/build

# Utility rule file for qualisys_gennodejs.

# Include the progress variables for this target.
include qualisys/CMakeFiles/qualisys_gennodejs.dir/progress.make

qualisys_gennodejs: qualisys/CMakeFiles/qualisys_gennodejs.dir/build.make

.PHONY : qualisys_gennodejs

# Rule to build all files generated by this target.
qualisys/CMakeFiles/qualisys_gennodejs.dir/build: qualisys_gennodejs

.PHONY : qualisys/CMakeFiles/qualisys_gennodejs.dir/build

qualisys/CMakeFiles/qualisys_gennodejs.dir/clean:
	cd /home/cristinaescribano/CrazyDrones/build/qualisys && $(CMAKE_COMMAND) -P CMakeFiles/qualisys_gennodejs.dir/cmake_clean.cmake
.PHONY : qualisys/CMakeFiles/qualisys_gennodejs.dir/clean

qualisys/CMakeFiles/qualisys_gennodejs.dir/depend:
	cd /home/cristinaescribano/CrazyDrones/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cristinaescribano/CrazyDrones/src /home/cristinaescribano/CrazyDrones/src/qualisys /home/cristinaescribano/CrazyDrones/build /home/cristinaescribano/CrazyDrones/build/qualisys /home/cristinaescribano/CrazyDrones/build/qualisys/CMakeFiles/qualisys_gennodejs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : qualisys/CMakeFiles/qualisys_gennodejs.dir/depend

