# include $(shell rospack find mk)/cmake.mk

#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug
#acceptable build_mode: Simulation/Hardware
build_mode=Simulation

.SILENT:

all: build_mode=Simulation
all: build build/CMakeLists.txt.copy
	$(info build_type is [${build_type}])
	$(info build_mode is [${build_mode}])
	$(MAKE) --no-print-directory -C build

hardware: build_mode=Hardware
hardware: build build/CMakeLists.txt.copy
	$(info build_type is [${build_type}])
	$(info build_mode is [${build_mode}])
	$(MAKE) --no-print-directory -C build

omega: build_mode=Omega
omega: build build/CMakeLists.txt.copy
	$(info build_type is [${build_type}])
	$(info build_mode is [${build_mode}])
	$(MAKE) --no-print-directory -C build

clean:
	rm -rf build bin lib msg_gen src/ut_automata

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) -DCMAKE_BUILD_MODE=$(build_mode) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build
