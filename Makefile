# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_COMMAND = /home/jungwon/clion-2019.1.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/jungwon/clion-2019.1.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jungwon/catkin_ws/src/swarm_simulator/swarm_planner/third_party/AA-SIPP-m

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jungwon/catkin_ws/src/swarm_simulator/swarm_planner/third_party/AA-SIPP-m

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/home/jungwon/clion-2019.1.4/bin/cmake/linux/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/home/jungwon/clion-2019.1.4/bin/cmake/linux/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/jungwon/catkin_ws/src/swarm_simulator/swarm_planner/third_party/AA-SIPP-m/CMakeFiles /home/jungwon/catkin_ws/src/swarm_simulator/swarm_planner/third_party/AA-SIPP-m/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/jungwon/catkin_ws/src/swarm_simulator/swarm_planner/third_party/AA-SIPP-m/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named sipp

# Build rule for target.
sipp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 sipp
.PHONY : sipp

# fast build rule for target.
sipp/fast:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/build
.PHONY : sipp/fast

src/aa_sipp.o: src/aa_sipp.cpp.o

.PHONY : src/aa_sipp.o

# target to build an object file
src/aa_sipp.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/aa_sipp.cpp.o
.PHONY : src/aa_sipp.cpp.o

src/aa_sipp.i: src/aa_sipp.cpp.i

.PHONY : src/aa_sipp.i

# target to preprocess a source file
src/aa_sipp.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/aa_sipp.cpp.i
.PHONY : src/aa_sipp.cpp.i

src/aa_sipp.s: src/aa_sipp.cpp.s

.PHONY : src/aa_sipp.s

# target to generate assembly for a file
src/aa_sipp.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/aa_sipp.cpp.s
.PHONY : src/aa_sipp.cpp.s

src/config.o: src/config.cpp.o

.PHONY : src/config.o

# target to build an object file
src/config.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/config.cpp.o
.PHONY : src/config.cpp.o

src/config.i: src/config.cpp.i

.PHONY : src/config.i

# target to preprocess a source file
src/config.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/config.cpp.i
.PHONY : src/config.cpp.i

src/config.s: src/config.cpp.s

.PHONY : src/config.s

# target to generate assembly for a file
src/config.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/config.cpp.s
.PHONY : src/config.cpp.s

src/constraints.o: src/constraints.cpp.o

.PHONY : src/constraints.o

# target to build an object file
src/constraints.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/constraints.cpp.o
.PHONY : src/constraints.cpp.o

src/constraints.i: src/constraints.cpp.i

.PHONY : src/constraints.i

# target to preprocess a source file
src/constraints.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/constraints.cpp.i
.PHONY : src/constraints.cpp.i

src/constraints.s: src/constraints.cpp.s

.PHONY : src/constraints.s

# target to generate assembly for a file
src/constraints.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/constraints.cpp.s
.PHONY : src/constraints.cpp.s

src/dynamicobstacles.o: src/dynamicobstacles.cpp.o

.PHONY : src/dynamicobstacles.o

# target to build an object file
src/dynamicobstacles.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/dynamicobstacles.cpp.o
.PHONY : src/dynamicobstacles.cpp.o

src/dynamicobstacles.i: src/dynamicobstacles.cpp.i

.PHONY : src/dynamicobstacles.i

# target to preprocess a source file
src/dynamicobstacles.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/dynamicobstacles.cpp.i
.PHONY : src/dynamicobstacles.cpp.i

src/dynamicobstacles.s: src/dynamicobstacles.cpp.s

.PHONY : src/dynamicobstacles.s

# target to generate assembly for a file
src/dynamicobstacles.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/dynamicobstacles.cpp.s
.PHONY : src/dynamicobstacles.cpp.s

src/main.o: src/main.cpp.o

.PHONY : src/main.o

# target to build an object file
src/main.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/main.cpp.o
.PHONY : src/main.cpp.o

src/main.i: src/main.cpp.i

.PHONY : src/main.i

# target to preprocess a source file
src/main.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/main.cpp.i
.PHONY : src/main.cpp.i

src/main.s: src/main.cpp.s

.PHONY : src/main.s

# target to generate assembly for a file
src/main.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/main.cpp.s
.PHONY : src/main.cpp.s

src/map.o: src/map.cpp.o

.PHONY : src/map.o

# target to build an object file
src/map.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/map.cpp.o
.PHONY : src/map.cpp.o

src/map.i: src/map.cpp.i

.PHONY : src/map.i

# target to preprocess a source file
src/map.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/map.cpp.i
.PHONY : src/map.cpp.i

src/map.s: src/map.cpp.s

.PHONY : src/map.s

# target to generate assembly for a file
src/map.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/map.cpp.s
.PHONY : src/map.cpp.s

src/mission.o: src/mission.cpp.o

.PHONY : src/mission.o

# target to build an object file
src/mission.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/mission.cpp.o
.PHONY : src/mission.cpp.o

src/mission.i: src/mission.cpp.i

.PHONY : src/mission.i

# target to preprocess a source file
src/mission.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/mission.cpp.i
.PHONY : src/mission.cpp.i

src/mission.s: src/mission.cpp.s

.PHONY : src/mission.s

# target to generate assembly for a file
src/mission.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/mission.cpp.s
.PHONY : src/mission.cpp.s

src/task.o: src/task.cpp.o

.PHONY : src/task.o

# target to build an object file
src/task.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/task.cpp.o
.PHONY : src/task.cpp.o

src/task.i: src/task.cpp.i

.PHONY : src/task.i

# target to preprocess a source file
src/task.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/task.cpp.i
.PHONY : src/task.cpp.i

src/task.s: src/task.cpp.s

.PHONY : src/task.s

# target to generate assembly for a file
src/task.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/task.cpp.s
.PHONY : src/task.cpp.s

src/tinyxml2.o: src/tinyxml2.cpp.o

.PHONY : src/tinyxml2.o

# target to build an object file
src/tinyxml2.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/tinyxml2.cpp.o
.PHONY : src/tinyxml2.cpp.o

src/tinyxml2.i: src/tinyxml2.cpp.i

.PHONY : src/tinyxml2.i

# target to preprocess a source file
src/tinyxml2.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/tinyxml2.cpp.i
.PHONY : src/tinyxml2.cpp.i

src/tinyxml2.s: src/tinyxml2.cpp.s

.PHONY : src/tinyxml2.s

# target to generate assembly for a file
src/tinyxml2.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/tinyxml2.cpp.s
.PHONY : src/tinyxml2.cpp.s

src/xmlLogger.o: src/xmlLogger.cpp.o

.PHONY : src/xmlLogger.o

# target to build an object file
src/xmlLogger.cpp.o:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/xmlLogger.cpp.o
.PHONY : src/xmlLogger.cpp.o

src/xmlLogger.i: src/xmlLogger.cpp.i

.PHONY : src/xmlLogger.i

# target to preprocess a source file
src/xmlLogger.cpp.i:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/xmlLogger.cpp.i
.PHONY : src/xmlLogger.cpp.i

src/xmlLogger.s: src/xmlLogger.cpp.s

.PHONY : src/xmlLogger.s

# target to generate assembly for a file
src/xmlLogger.cpp.s:
	$(MAKE) -f CMakeFiles/sipp.dir/build.make CMakeFiles/sipp.dir/src/xmlLogger.cpp.s
.PHONY : src/xmlLogger.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... sipp"
	@echo "... edit_cache"
	@echo "... src/aa_sipp.o"
	@echo "... src/aa_sipp.i"
	@echo "... src/aa_sipp.s"
	@echo "... src/config.o"
	@echo "... src/config.i"
	@echo "... src/config.s"
	@echo "... src/constraints.o"
	@echo "... src/constraints.i"
	@echo "... src/constraints.s"
	@echo "... src/dynamicobstacles.o"
	@echo "... src/dynamicobstacles.i"
	@echo "... src/dynamicobstacles.s"
	@echo "... src/main.o"
	@echo "... src/main.i"
	@echo "... src/main.s"
	@echo "... src/map.o"
	@echo "... src/map.i"
	@echo "... src/map.s"
	@echo "... src/mission.o"
	@echo "... src/mission.i"
	@echo "... src/mission.s"
	@echo "... src/task.o"
	@echo "... src/task.i"
	@echo "... src/task.s"
	@echo "... src/tinyxml2.o"
	@echo "... src/tinyxml2.i"
	@echo "... src/tinyxml2.s"
	@echo "... src/xmlLogger.o"
	@echo "... src/xmlLogger.i"
	@echo "... src/xmlLogger.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

