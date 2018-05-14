#include "bCollision.h"
/*
bool bCollision::AABBvsAABB(AABB a, AABB b)
{
	// Exit with no intersection if found separated along an axis
	if (a.max.x < b.min.x || a.min.x > b.max.x)
	{
		return false;
	}
	if (a.max.y < b.min.y || a.min.y > b.max.y)
	{
		return false;
	}
	// No separating axis found, therefor there is at least one overlapping axis
	return true;
}*/

float bCollision::Distance(glm::vec3 a, glm::vec3 b)
{
	return glm::sqrt(glm::pow((a.x - b.x), 2) + glm::pow((a.y - b.y),2));
}
/*
bool bCollision::CirclevsCircleUnoptimized(Circle a, Circle b)
{
	float r = a.radius + b.radius;
	return r < Distance(a.position, b.position);
}

bool bCollision::CirclevsCircleOptimized(Circle a, Circle b)
{
	float r = a.radius + b.radius;
	r *= r;
	return r < glm::pow((a.position.x + b.position.x), 2) + glm::pow((a.position.y + b.position.y),2);
}*/

bCollision::CollisionCallback bCollision::dispatch[Shape::count][Shape::count] = 
{
	{
		CircletoCircle,
		CircletoPolygon
	}
};

void bCollision::PositionalCorrection(bManifold* m, bRigidbody2D* A, bRigidbody2D* B)
{
	const float percent = 0.2; // usually 20% to 80%
	const float slop = 0.01; // usually 0.01 to 0.1
	glm::vec3 correction = glm::max(m->penetration - slop, 0.0f) / (A->inv_mass + B->inv_mass) * percent * m->normal;
	A->getParent()->myTransform->position -= A->inv_mass * correction;
	B->getParent()->myTransform->position += B->inv_mass * correction;
}

void bCollision::ResolveCollision(bManifold* m, bRigidbody2D* A, bRigidbody2D* B)
{
	
	// Calculate relative velocity
	glm::vec3 rv = B->velocity - A->velocity;

		// Calculate relative velocity in terms of the normal direction
	float velAlongNormal = glm::dot(rv, m->normal);

	// Do not resolve if velocities are separating
	if (velAlongNormal < 0)
	{
		// Calculate restitution
		float e = glm::min(A->restitution, B->restitution);

		// Calculate impulse scalar
		float j = -(1 + e) * velAlongNormal;
		j /= 1 / A->mass + 1 / B->mass;

		// Apply impulse
		glm::vec3 impulse = j * m->normal;
		A->velocity -= 1 / A->mass * impulse;
		B->velocity += 1 / B->mass * impulse;
	}
}

bool bCollision::CircletoCircle(bManifold *m, bRigidbody2D *a, bRigidbody2D *b)
{
	bCircle *A = reinterpret_cast<bCircle *>(a->shape);
	bCircle *B = reinterpret_cast<bCircle *>(b->shape);

	glm::vec3 normal = b->getParent()->myTransform->position - a->getParent()->myTransform->position;

	float r = A->getRadius() + B->getRadius();
	float temp = glm::length2(normal);
	//std::cout << "temp: " << temp << "r" << r << std::endl;
	if (glm::length2(normal) > glm::pow(r, 2))
	{
		m->contact_count = 0;
		return false;
	}

	// Circles have collided, compute manifold.
	float distance = std::sqrt(glm::length2(normal));

	//std::cout << "Circle collission" << std::endl;
	// If distance between circles is not zero
	if (distance != 0)
	{
		// Penetration Distance is difference between radius and distance
		m->penetration = r - distance;
		//m->contact_points.push_back(A->body->getParent()->myTransform->position);
		//m->contact_points.push_back(B->body->getParent()->myTransform->position);
		// Utilize our distance since we performed sqrt on it already within Length( )
		// Points from A to B, and is a unit vector
		m->contact_count = 1;
		m->normal = normal / distance;
		m->isColliding = true;
		return true;
	}
	else // circles are on the same position (What does this mean?)
	{
		//m->contact_points.push_back(A->body->getParent()->myTransform->position);
		//m->contact_points.push_back(B->body->getParent()->myTransform->position);
		m->penetration = A->getRadius();
		m->normal = glm::vec3(1, 0, 0);
		//m->contact_count = 1;
		m->isColliding = true;
		return true;
	}
}

bool bCollision::CircletoPolygon(bManifold *m, bRigidbody2D *a, bRigidbody2D *b)
{
	
	bCircle *A = reinterpret_cast<bCircle *>(a->shape);
	bPoly *B = reinterpret_cast<bPoly *>(b->shape);

	//m->contact_count = 0;

	// Vector from A to B
	glm::vec3 normal = b->getParent()->myTransform->position - a->getParent()->myTransform->position;

	// Closest point on A to center of B
	glm::vec3 closest = normal;

	// Calculate half extents along each axis
	float x_extent = (B->max.x - B->min.x) / 2;
	float y_extent = (B->max.y - B->min.y) / 2;

	// Clamp point to edges of the AABB
	//closest.x = glm::clamp(-x_extent, x_extent, closest.x);
	//closest.y = glm::clamp(-y_extent, y_extent, closest.y);
	closest.x = glm::clamp(closest.x, -x_extent, x_extent);
	closest.y = glm::clamp(closest.y, -y_extent, y_extent);

	bool inside = false;

	// Circle is inside the AABB, so we need to clamp the circle's center
	// to the closest edge
	if (normal == closest)
	{
		inside = true;

		// Find closest axis
		if (glm::abs(normal.x) > glm::abs(normal.y))
		{
			// Clamp to closest extent
			if (closest.x > 0)
			{
				closest.x = x_extent;
			}
			else
			{
				closest.x = -x_extent;
			}

		}
		else
		{
			// y axis is shorter
			// Clamp to closest extent
			if (closest.y > 0)
				closest.y = y_extent;
			else
				closest.y = -y_extent;
		}
	}

	glm::vec3 normalizedNormal = normal - closest;
	float distance = glm::length2(normalizedNormal);
	//float distance = (normalizedNormal.x * normalizedNormal.x) + (normalizedNormal.y * normalizedNormal.y);
	
	float radius = A->getRadius();

	// Early out of the radius is shorter than distance to closest point and
	// Circle not inside the AABB
	if ((distance > std::pow(radius,2)) && !inside)
		return false;

	// Avoided sqrt until we needed
	distance = std::sqrt(distance);

	//m->contact_count = 1;

	// Collision normal needs to be flipped to point outside if circle was
	// inside the AABB
	if (inside)
	{
		//m->contact_points.push_back(A->body->getParent()->myTransform->position);
		//m->contact_points.push_back(B->body->getParent()->myTransform->position);
		m->normal = -normalizedNormal / distance;
		m->penetration = radius - distance;
	}
	else
	{
		//m->contact_points.push_back(A->body->getParent()->myTransform->position);
		//m->contact_points.push_back(B->body->getParent()->myTransform->position);
		m->normal = normalizedNormal / distance;
		m->penetration = radius - distance;
	}

	//m->contact_count = 1;
	m->isColliding = true;


	return true;

}