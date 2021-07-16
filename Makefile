#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

.SILENT:

all: build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

# Sets the build type to Debug.
set_debug:
	$(eval build_type=Debug)

# Ensures that the build type is debug before running all target.
debug_all: | set_debug all

clean:
	rm -rf build bin lib

build/CMakeLists.txt.copy: CMakeLists.txt Makefile srv
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy
