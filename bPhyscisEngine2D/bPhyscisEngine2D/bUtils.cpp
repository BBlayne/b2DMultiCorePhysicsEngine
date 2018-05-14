#include "bUtils.h"
#include <iostream>

void bUtils::measureFPS()
{
	// Measure speed
	double currentTime = glfwGetTime();
	nbFrames++;
	if (currentTime - lastTime >= 1.0) { // If last prinf() was more than 1sec ago
		// printf and reset
		printf("%f ms/frame\n", 1000.0 / double(nbFrames));
		nbFrames = 0;
		lastTime += 1.0;
	}
}

bUtils::bUtils()
{
	this->nbFrames = 0;
	this->lastTime = 0;
}

std::vector<glm::vec3> bUtils::ArrayToVertices(std::vector<float> _vertices)
{
	// Need error checking
	std::vector<glm::vec3> mVertices;
	for (int i = 0; i < _vertices.size(); i+=3)
	{	
		mVertices.push_back(glm::vec3(i, i + 1, i + 2));
	}

	return mVertices;
}

std::vector<glm::vec3> bUtils::ArrayToVertices(const GLfloat* _vertices, int size)
{
	// Need error checking
	std::vector<glm::vec3> mVertices;
	for (int i = 0; i < size; i += 3)
	{
		mVertices.push_back(glm::vec3(_vertices[i], _vertices[i + 1], _vertices[i + 2]));
	}

	return mVertices;
}

float bUtils::remap(float value, float from1, float to1, float from2, float to2)
{
	return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
}

glm::vec3 bUtils::ScreenToWorldSpaceCoords(int x, int y, int screen_width, int screen_height)
{
	float world_x = x - (screen_width / 2);
	float world_y = y - (screen_height);
	world_y = -y + (screen_height / 2);

	glm::vec3 pos = glm::vec3(world_x, world_y, 0);

	return pos;
}

glm::vec3 bUtils::ScreenToWorldSpaceCoords(glm::vec3 _xy, int screen_width, int screen_height)
{
	float world_x = _xy.x - (screen_width / 2);
	float world_y = _xy.y - (screen_height);
	world_y = -_xy.y + (screen_height / 2);

	glm::vec3 pos = glm::vec3(world_x, world_y, 0);

	return pos;
}

glm::vec3 bUtils::HalfOffsetScreenToScreenCoords(float _x, float _y, int screen_width, int screen_height)
{
	// Need to correct for aspect ratio
	float aspect = (float)screen_width / screen_height;
	float begin_1, begin_2, end_1, end_2;
	begin_1 = (screen_width / 2);
	begin_2 = 0;
	end_1 = screen_width / 2;
	end_2 = screen_width;

	float screen_x = remap(_x * aspect, -begin_1, end_1, begin_2, end_2);

	begin_1 = (screen_height / 2);
	begin_2 = 0;
	end_1 = screen_height / 2;
	end_2 = screen_height;
	float screen_y = remap(_y * aspect, begin_1, -end_1, begin_2, end_2);

	glm::vec3 pos = glm::vec3(screen_x, screen_y, 0);

	return pos;
}