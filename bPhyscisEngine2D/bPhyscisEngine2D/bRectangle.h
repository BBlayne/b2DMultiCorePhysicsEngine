#pragma once
#include "Shape.h"
#include "bRigidBody2D.h"

class bPoly : public Shape {
private:
	float width;
	float height;
	void ComputeMass(float density, std::vector<glm::vec3> vertices);
public:
	Type GetType();
	void Init();	
	glm::vec3 min;
	glm::vec3 max;

	void setAABB(glm::vec3 position, float width, float height);
	void SetOrient(float radians);
	void UpdateExtents(float scale);
	glm::vec2 GetExtents();

	bPoly();
	bPoly(float density, std::vector<glm::vec3> vertices);
	bPoly(bRigidbody2D* _body, glm::vec3 _pos, float _scaleWidth, float _scaleHeight, float density, std::vector<glm::vec3> vertices);
	~bPoly() {};
};