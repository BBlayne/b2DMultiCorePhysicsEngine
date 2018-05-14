#pragma once
#include "Dependencies\glm\glm\glm.hpp"

class bRect {
private:
	int x;
	int y;
	int width;
	int height;
public:
	int getX() { return x; };
	int getY() { return y; };
	int getWidth() { return width; };
	int getHeight() { return height; };
	bRect(int _x, int _y, int _width, int _height) {
		x = _x;
		y = _y;
		width = _width;
		height = _height;
	};
	bRect(glm::vec2 pos, glm::vec2 bounds) {
		x = pos.x;
		y = pos.y;
		width = bounds.x;
		height = bounds.y;
	};

	~bRect() {};
};