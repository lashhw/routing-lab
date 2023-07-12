all: router

router: main.cpp LayerAssignment.cpp
	g++ -O3 -std=c++17 main.cpp LayerAssignment.cpp -o router

.PHONY: clean
clean:
	rm router
