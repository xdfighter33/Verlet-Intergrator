#include <iostream>
#include <stdint.h>

class Grids{
private:
	int width;
	int height; 
	std::vector<std::vector<char>> grid;


public: 

	Grids(int w, int h) : width(w), height(h), grid(h, std::vector<char>(w, '.')) {}
};