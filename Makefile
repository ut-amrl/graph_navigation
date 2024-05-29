#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

.SILENT:

all: build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build


# fix the docker commands
docker_all: docker_build_q
	docker run --rm --volume "$(shell pwd)":/home/dev/graph_navigation graph_navigation "cd graph_navigation && make -j"

docker_shell: docker_build_q
	if [ $(shell docker ps -a -f name=graph_navigation_shell | wc -l) -ne 2 ]; then xhost +local:root; docker run -dit --name graph_navigation_shell --volume "$(shell pwd)":/home/dev/graph_navigation --workdir /home/dev/graph_navigation -p 10272:10272 -e DISPLAY=$(DISPLAY) -v /tmp/.X11-unix:/tmp/.X11-unix graph_navigation; fi
	docker exec -it graph_navigation_shell bash -l

docker_stop:
	docker container stop graph_navigation_shell
	docker container rm graph_navigation_shell

docker_build:
	docker build --build-arg HOST_UID=$(shell id -u) -t graph_navigation .

docker_build_q:
	docker build -q --build-arg HOST_UID=$(shell id -u) -t graph_navigation .

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
