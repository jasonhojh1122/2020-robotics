all: part_e_2

CFLAGS=-fPIC -g -Wall -std=c++98
ARIA_INCLUDE=-I/usr/local/Aria/include
ARIA_LINK=-L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt

%: %.cpp
	$(CXX) $(CFLAGS) $(ARIA_INCLUDE) $< -o $@ $(ARIA_LINK)
