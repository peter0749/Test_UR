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
CMAKE_SOURCE_DIR = /home/demo/tm700_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/demo/tm700_ws/build

# Include any dependencies generated for this target.
include industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/depend.make

# Include the progress variables for this target.
include industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/progress.make

# Include the compile flags for this target's objects.
include industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/flags.make

industrial_core/simple_message/simple_message_dummy.cpp:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/demo/tm700_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating simple_message_dummy.cpp"
	cd /home/demo/tm700_ws/build/industrial_core/simple_message && /usr/bin/cmake -E touch /home/demo/tm700_ws/build/industrial_core/simple_message/simple_message_dummy.cpp

industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o: industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/flags.make
industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o: industrial_core/simple_message/simple_message_dummy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/demo/tm700_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o"
	cd /home/demo/tm700_ws/build/industrial_core/simple_message && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o -c /home/demo/tm700_ws/build/industrial_core/simple_message/simple_message_dummy.cpp

industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.i"
	cd /home/demo/tm700_ws/build/industrial_core/simple_message && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/demo/tm700_ws/build/industrial_core/simple_message/simple_message_dummy.cpp > CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.i

industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.s"
	cd /home/demo/tm700_ws/build/industrial_core/simple_message && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/demo/tm700_ws/build/industrial_core/simple_message/simple_message_dummy.cpp -o CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.s

industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o.requires:

.PHONY : industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o.requires

industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o.provides: industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o.requires
	$(MAKE) -f industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/build.make industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o.provides.build
.PHONY : industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o.provides

industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o.provides.build: industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o


# Object files for target simple_message_dummy
simple_message_dummy_OBJECTS = \
"CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o"

# External object files for target simple_message_dummy
simple_message_dummy_EXTERNAL_OBJECTS =

/home/demo/tm700_ws/devel/lib/libsimple_message_dummy.so: industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o
/home/demo/tm700_ws/devel/lib/libsimple_message_dummy.so: industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/build.make
/home/demo/tm700_ws/devel/lib/libsimple_message_dummy.so: industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/demo/tm700_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library /home/demo/tm700_ws/devel/lib/libsimple_message_dummy.so"
	cd /home/demo/tm700_ws/build/industrial_core/simple_message && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple_message_dummy.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/build: /home/demo/tm700_ws/devel/lib/libsimple_message_dummy.so

.PHONY : industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/build

industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/requires: industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/simple_message_dummy.cpp.o.requires

.PHONY : industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/requires

industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/clean:
	cd /home/demo/tm700_ws/build/industrial_core/simple_message && $(CMAKE_COMMAND) -P CMakeFiles/simple_message_dummy.dir/cmake_clean.cmake
.PHONY : industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/clean

industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/depend: industrial_core/simple_message/simple_message_dummy.cpp
	cd /home/demo/tm700_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/demo/tm700_ws/src /home/demo/tm700_ws/src/industrial_core/simple_message /home/demo/tm700_ws/build /home/demo/tm700_ws/build/industrial_core/simple_message /home/demo/tm700_ws/build/industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : industrial_core/simple_message/CMakeFiles/simple_message_dummy.dir/depend

