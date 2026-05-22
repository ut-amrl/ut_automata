build_type ?= Release
colcon_args ?= -DPython3_EXECUTABLE=/usr/bin/python3
base_paths ?= . ../amrl_msgs ../amrl_maps
parallel_workers ?= $(shell nproc)

.PHONY: all simulation hardware clean

all: simulation

simulation:
	CMAKE_BUILD_PARALLEL_LEVEL=$(parallel_workers) colcon build --symlink-install --parallel-workers $(parallel_workers) --base-paths $(base_paths) --packages-up-to ut_automata --cmake-args -DCMAKE_BUILD_TYPE=$(build_type) $(colcon_args)

hardware: simulation

clean:
	rm -rf build install log
