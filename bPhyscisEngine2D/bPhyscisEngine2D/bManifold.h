#pragma once
#include "Dependencies\glm\glm\glm.hpp"
#include "bRigidbody2D.h"


class bManifold {
	float sf;
	float df;
public:
	bManifold(bRigidbody2D* A, bRigidbody2D* B);

	bRigidbody2D* A;
	bRigidbody2D* B;
	float penetration;
	float e;
	int contact_count;	   // Number of contacts that occured during collision
	std::vector<glm::vec3> contact_points; // Points of contact during collision
	glm::vec3 normal;      // From A to B
	void ApplyImpulse();
	void ApplyImpulseSimplified();
	void Initialize();
	void Solve();
	void InfiniteMassCorrection();
	bool isColliding;
};