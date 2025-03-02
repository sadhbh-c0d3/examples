all: debug release

rebuild: clean all

clean:
	rm -rf ../bin

release:
	mkdir -p ../bin/release
	cd ../bin/release && g++ --std=c++20 -stdlib=libc++ -fexperimental-library ../../p1_cpp_nodegraph/*.cpp -o p1_cpp_nodegraph

DEBUG_FLAGS=-DDBG_ENABLE=1 -DDBG_ENABLE_RWLOCK=1 -DDBG_ENABLE_ACTION=1

debug:
	mkdir -p ../bin/debug
	g++ --std=c++20 -stdlib=libc++ -fexperimental-library $(DEBUG_FLAGS) *.cpp -o ../bin/debug/p1_cpp_nodegraph


run:
	cd ../bin/release && ./p1_cpp_nodegraph

run_debug:
	cd ../bin/debug && ./p1_cpp_nodegraph
