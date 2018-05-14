#pragma once
#include "Shape.h"
#include "bRigidBody2D.h"

class bCircle : public Shape {
private:
	void ComputeMass(float density);
	float radius;
public:
	Type GetType();
	void Init();
	void SetOrient(float radians);
	void UpdateExtents(float scale);
	glm::vec2 GetExtents();
	void setRadius(float _radius) { this->radius = _radius; };
	float getRadius() { return this->radius; };
	bCircle();
	~bCircle() {};
	bCircle(bRigidbody2D* _body, glm::vec3 _pos, float _radius, float density);
};