#include "bManifold.h"
#include "bCollision.h"
#include "bPhysics.h"
#include <iostream>

bPhysics physicsInfo;

bManifold::bManifold(bRigidbody2D* a, bRigidbody2D* b)
{
	isColliding = false;
	this->A = a;
	this->B = b;
}

void bManifold::Solve()
{
	//|| B->shape->GetType() == 0
	if (A->shape->GetType() == 1 )
	{
		//std::cout << "A: Rectangle" << std::endl;
	}
	else
	{
		bCollision::dispatch[A->shape->GetType()][B->shape->GetType()](this, A, B);
	}
}

void bManifold::InfiniteMassCorrection()
{
	A->velocity = glm::vec3(0, 0, 0);
	B->velocity = glm::vec3(0, 0, 0);
}

void bManifold::ApplyImpulseSimplified()
{
	glm::vec3 rv = B->velocity - A->velocity;
	float contactVel = glm::dot(rv, normal);
	if (contactVel > 0)
		return;

	float e = glm::min(A->restitution, B->restitution);

	float numerator = -(1.0f + e) * contactVel;
	float j = numerator / (A->inv_mass + B->inv_mass);
	glm::vec3 impulse = j * normal * 1.0f;

	A->ApplyImpulse(-(impulse), (B->getParent()->myTransform->position - A->getParent()->myTransform->position));
	B->ApplyImpulse(impulse, (A->getParent()->myTransform->position - B->getParent()->myTransform->position));

}

void bManifold::ApplyImpulse()
{
	
	if ((A->inv_mass + B->inv_mass) == 0)
	{
		InfiniteMassCorrection();
		return;
	}

	// Calculate radii from COM to contact
	//glm::vec3 ra = contact_points[0] - A->getParent()->myTransform->position;
	//glm::vec3 rb = contact_points[1] - B->getParent()->myTransform->position;

	//relative velocity
	//glm::vec3 rv = B->velocity + glm::vec3((-B->angularVelocity * rb.y), (B->angularVelocity * rb.x), 0) -
	//	A->velocity - glm::vec3((-A->angularVelocity * ra.y), (A->angularVelocity * ra.x), 0);
	glm::vec3 rv = B->velocity - A->velocity;


	// Relative velocity along the normal
	float contactVel = glm::dot(rv, normal);

	if (contactVel > 0)
		return;

	//float raCrossN = ra.x * normal.y - ra.y * normal.x;
	//float rbCrossN = rb.x * normal.y - rb.y * normal.x;
	//float powRaCrossN = std::pow(raCrossN, 2);
	//float powrbCrossN = std::pow(rbCrossN, 2);
	float invMassSum = A->inv_mass + B->inv_mass; //+ powRaCrossN * A->inv_intertia + powrbCrossN * B->inv_intertia;

	// calculate inverse scalar
	float j = -(1.0f + e) * contactVel;
	j /= invMassSum;
	//j /= (float)contact_count;
	// apply impulse
	glm::vec3 impulse = normal * j;
	//A->ApplyImpulse(-impulse, ra);
	//B->ApplyImpulse(impulse, rb);

	/*
	for (int i = 0; i < contact_count; ++i)
	{
		// Calculate radii from COM to contact
		glm::vec3 ra = contact_points[0] - A->getParent()->myTransform->position;
		glm::vec3 rb = contact_points[1] - B->getParent()->myTransform->position;

		//relative velocity
		glm::vec3 rv = B->velocity + glm::vec3((-B->angularVelocity * rb.y), (B->angularVelocity * rb.x), 0) -
			A->velocity - glm::vec3((-A->angularVelocity * ra.y), (A->angularVelocity * ra.x), 0);

		// Relative velocity along the normal
		float contactVel = glm::dot(rv, normal);

		if (contactVel > 0)
			return;

		float raCrossN = ra.x * normal.y - ra.y * normal.x;
		float rbCrossN = rb.x * normal.y - rb.y * normal.x;
		float powRaCrossN = std::pow(raCrossN, 2);
		float powrbCrossN = std::pow(rbCrossN, 2);
		float invMassSum = A->inv_mass + B->inv_mass + powRaCrossN * A->inv_intertia + powrbCrossN * B->inv_intertia;

		// calculate inverse scalar
		float j = -(1.0f + e) * contactVel;
		j /= invMassSum;
		//j /= (float)contact_count;
		// apply impulse
		glm::vec3 impulse = normal * j;
		A->ApplyImpulse(-impulse, ra);
		B->ApplyImpulse(impulse, rb);

		// friction
	}
	*/
}

void bManifold::Initialize()
{
	// Calculate average restitution
	try {
		e = glm::min(A->restitution, B->restitution);
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception: Initialize: " << e.what() << std::endl;
	}
	

	// Calculate static and dynamic friction
	sf = std::sqrt(A->staticFriction * B->staticFriction);
	df = std::sqrt(A->dynamicFriction * B->dynamicFriction);
	/*
	for (int i = 0; i < contact_count; ++i)
	{
		// Calculate radii from COM to contact
		if (contact_points.size() < 2)
			continue;

		glm::vec3 ra = contact_points.at(0) - A->getParent()->myTransform->position;
		glm::vec3 rb = contact_points.at(1) - B->getParent()->myTransform->position;

		glm::vec3 rv = B->velocity + glm::vec3((-B->angularVelocity * rb.y), (B->angularVelocity * rb.x),0) -
					   A->velocity - glm::vec3((-A->angularVelocity * ra.y), (A->angularVelocity * ra.x), 0);


		// Determine if we should perform a resting collision or not
		// The idea is if the only thing moving this object is gravity,
		// then the collision should be performed without any restitution
		if (glm::length2(rv) < glm::length2(physicsInfo.dt * physicsInfo.gravity) + physicsInfo.EPSILON)
			e = 0.0f;
	}
	*/
}