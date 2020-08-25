# include $(shell rospack find mk)/cmake.mk

#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug
#acceptable build_mode: Simulation/Hardware
build_mode=Simulation
# directory of amrl_maps
map_dir = $(shell rospack find amrl_maps) 

.SILENT:

all: build_mode=Simulation
all: maps build build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(info Build_mode is [${build_mode}])
	$(MAKE) --no-print-directory -C build

hardware: build_mode=Hardware
hardware: maps build build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(info Build_mode is [${build_mode}])
	$(MAKE) --no-print-directory -C build

clean:
	rm -rf build bin lib msg_gen src/ut_automata maps

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) -DCMAKE_BUILD_MODE=$(build_mode) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

maps:
	ln -s $(map_dir) maps

build:
	mkdir -p build

purge: clean cleanup_cache
	rm -rf src/ut_automata

