all: interactive_demo.cc tests.cc
	$(CXX) --std=c++11 -Wextra -Wall -Werror -g -I ../include/ -o interactive_demo interactive_demo.cc -llua5.1 -lpthread
	$(CXX) --std=c++11 -Wextra -Wall -Werror -g -I ../include/ -o tests tests.cc -llua5.1 -lpthread

valgrind_demo: all
	valgrind --leak-check=full ./interactive_demo

run_tests: all
	./tests

valgrind_tests: all
	valgrind --leak-check=full ./tests

test: all run_tests valgrind_tests

clean: example
	rm example
