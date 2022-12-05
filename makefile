all: git_submodule main

main: main.cpp serial_port.cpp autopilot_interface.cpp cmd_interface_linux.cpp lipkg.cpp tofbf.cpp
	g++ -g -w -Wall -I mavlink/include/mavlink/v2.0 main.cpp serial_port.cpp autopilot_interface.cpp cmd_interface_linux.cpp lipkg.cpp tofbf.cpp -o main -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o main
