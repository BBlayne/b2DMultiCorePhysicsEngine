#pragma once
#include <iostream>
#include "Dependencies\glm\glm\glm.hpp"
#include "Dependencies\GL\glew.h"
#include "Dependencies\GL\freeglut.h"
#include "Dependencies\glfw3.h"
#include "Dependencies\Shaders\shader.hpp"
#include "Dependencies\glm\glm\gtc\matrix_transform.hpp"
#include "Dependencies\glm\glm\gtx\transform.hpp"
#include "Dependencies\glm\glm\gtc\quaternion.hpp"
#include "Dependencies\glm\glm\gtx\quaternion.hpp"
#include "bGLOBALS.h"
#include "Transform.h"
#include "Shape.h"

class bWidget;

#include "Material.h"
#include "Mesh.h"

class bRigidbody2D {
private:
	bWidget* parent;
	float density;
public:
	Shape* shape;
	Transform* transform;
	Material material;
	float inv_intertia;
	float inertia;
	float inv_mass;
	float mass;
	glm::vec3 force;
	glm::vec3 velocity;
	float angularVelocity;
	float torque;
	float impulse;
	float restitution;
	float staticFriction;
	float dynamicFriction;

	// methods
	void ApplyImpulse(const glm::vec3& impulse, const glm::vec3& contactVector);
	void SetStatic();
	// setters & getters
	void setParent(bWidget* _parent) { this->parent = _parent; };
	bWidget* getParent() { return this->parent; };

	void setDensity(float _density) { this->density = _density;	};
	float getDensity() { return this->density; };

	bRigidbody2D();
	bRigidbody2D(bWidget* _parent);
	bRigidbody2D(bWidget* _parent, float restitution, bool IsStatic);
	bRigidbody2D(float restitution, bool IsStatic);

};