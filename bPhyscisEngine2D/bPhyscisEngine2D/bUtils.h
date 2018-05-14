#include <vector>
#include "Dependencies\glm\glm\glm.hpp"
#include "Dependencies\glfw3.h"

class bUtils
{
private:
	int nbFrames;
	int lastTime;
public:
	bUtils();
	void measureFPS();
	void setLastTime(int _time) { this->lastTime = _time; };
	std::vector<glm::vec3> ArrayToVertices(std::vector<float> _vertices);
	std::vector<glm::vec3> ArrayToVertices(const GLfloat* _vertices, int size);
	float remap(float value, float from1, float to1, float from2, float to2);
	glm::vec3 ScreenToWorldSpaceCoords(int x, int y, int screen_width, int screen_height);
	glm::vec3 ScreenToWorldSpaceCoords(glm::vec3 _xy, int screen_width, int screen_height);
	glm::vec3 HalfOffsetScreenToScreenCoords(float _x, float _y, int screen_width, int screen_height);
};