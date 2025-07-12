# Graph Navigation ROS2 Makefile
# Traditional cmake build pattern following webviz/enml style

SHELL = /bin/bash

#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

.SILENT:

all: install

# Install target for ROS2
install: build-only
	echo "ROS2 detected, installing to ./install ..."
	$(MAKE) --no-print-directory -C build install

# Build-only target (no install)
build-only: build build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

# Sets the build type to Debug.
set_debug:
	$(eval build_type=Debug)

# Ensures that the build type is debug before running all target.
debug_all: | set_debug all

clean:
	rm -rf build bin lib install log

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) -DCMAKE_INSTALL_PREFIX=../install ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build

# Additional convenience targets
test: build-only
	cd build && make test

help:
	@echo "Available targets:"
	@echo "  all        - Build and install the package"
	@echo "  build-only - Build without installing"
	@echo "  install    - Install after building"
	@echo "  clean      - Clean build artifacts"
	@echo "  test       - Run tests"
	@echo "  help       - Show this help message"
	@echo "  debug_all  - Build in debug mode"

# Backwards compatibility
deps:
	@echo "Installing dependencies..."
	./InstallPackages
