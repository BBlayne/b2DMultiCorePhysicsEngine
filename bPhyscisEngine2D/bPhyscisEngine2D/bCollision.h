#pragma once
#define FLT_MAX 3.40282347E+38F;
#include "Dependencies\glm\glm\glm.hpp"
#include "Dependencies\glm\glm\gtx\norm.hpp"
#include "bWidget.h"
#include "bRigidbody2D.h"
#include "Shape.h"
#include "bCircle.h"
#include "bRectangle.h"
#include "bManifold.h"
#include <functional>

class bCollision {
private:
	float Distance(glm::vec3 a, glm::vec3 b);
	static void A0(void) { };
public:
	bManifold* manifold;

	void ResolveCollision(bManifold* m, bRigidbody2D* A, bRigidbody2D* B);
	void PositionalCorrection(bManifold* m, bRigidbody2D* A, bRigidbody2D* B);
	
	typedef bool(*CollisionCallback)(bManifold *m, bRigidbody2D *a, bRigidbody2D *b);

	static CollisionCallback dispatch[Shape::count][Shape::count];
	static bool CircletoCircle(bManifold *m, bRigidbody2D *a, bRigidbody2D *b);
	static bool CircletoPolygon(bManifold *m, bRigidbody2D *a, bRigidbody2D *b);
	static bool CircletoPolygonSimplified(bManifold *m, bRigidbody2D *a, bRigidbody2D *b);
};
