#pragma once
#include "Dependencies\glm\glm\glm.hpp"

class bRigidbody2D;

class Transform {
private:
public:
	bRigidbody2D* mBody;
	glm::vec3 position;
	float orientation_in_radians;
	glm::mat4 translation;
	glm::mat4 rotation;
	glm::mat4 scale;

	void SetOrient(float radians);
	void setScale(glm::vec3 _scale);

	Transform();
	Transform(float orientation_radians);
	Transform(float orientation_radians, glm::vec3 pos);
	~Transform() {};
};