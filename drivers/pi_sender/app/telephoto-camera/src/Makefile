CXX ?= g++
CXXFLAGS ?= -O3 -DNDEBUG -std=c++17 -Wall -Wextra -Wpedantic

.PHONY: all clean

all: telephoto_udp_sender

telephoto_udp_sender: telephoto_udp_sender.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -f telephoto_udp_sender
