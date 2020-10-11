
CFLAGS=-fPIC -g -Wall -std=c++11
ARIA_INCLUDE=-I/usr/local/Aria/include
ARIA_LINK=-L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt

c:
	$(CXX) part_c.cpp $(CFLAGS) $(ARIA_INCLUDE) -o part_c $(ARIA_LINK)

d: 
	$(CXX) part_d.cpp $(CFLAGS) $(ARIA_INCLUDE) -o part_d $(ARIA_LINK)

e: 
	$(CXX) part_e.cpp $(CFLAGS) $(ARIA_INCLUDE) -o part_e $(ARIA_LINK)

b: 
	$(CXX) bonus.cpp $(CFLAGS) $(ARIA_INCLUDE) -o bonus $(ARIA_LINK)

clean:
	rm -f part_c part_d part_e bonus