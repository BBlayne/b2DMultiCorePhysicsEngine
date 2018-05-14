#pragma once
#include "Dependencies\glm\glm\glm.hpp"

class bRigidbody2D;

class Shape {

protected:

public:
	bRigidbody2D* body;

	enum Type
	{
		circle,
		poly,
		count
	};
	virtual Type GetType(void) = 0;
	virtual void SetOrient(float radians) = 0;
	virtual void UpdateExtents(float scale) = 0;
	virtual glm::vec2 GetExtents(void) = 0;
	glm::mat4 orientation = glm::mat4(1.0f);
};