#include "bRigidbody2D.h"
#include "Dependencies\Texture\texture.hpp"

using namespace glm;

bRigidbody2D::bRigidbody2D()
{
	this->force = glm::vec3(0, 0, 0);
	this->torque = 0.0f;
	this->density = 1.0f;
	this->angularVelocity = 0.0f;
	this->dynamicFriction = 0.0f;
	this->impulse = 0.0f;
	this->inertia = 0.0f;
	this->inv_intertia = 0.0f;
	this->inv_mass = 0.0f;
	this->mass = 0.0f;
	this->velocity = glm::vec3(0, 0, 0);
	this->staticFriction = 0.0f;
	this->restitution = CIRCLE_BOUNCE;
}

bRigidbody2D::bRigidbody2D(bWidget* _parent)
{
	this->force = glm::vec3(0, 0, 0);
	this->torque = 0.0f;
	this->density = 1.0f;
	this->angularVelocity = 0.0f;
	this->dynamicFriction = 0.0f;
	this->impulse = 0.0f;
	this->inertia = 0.0f;
	this->inv_intertia = 0.0f;
	this->inv_mass = 0.0f;
	this->mass = 0.0f;
	this->velocity = glm::vec3(0, 0, 0);
	this->staticFriction = 0.0f;
	this->restitution = restitution;
	this->setParent(_parent);
}

bRigidbody2D::bRigidbody2D(bWidget* _parent, float restitution, bool IsStatic)
{
	this->force = glm::vec3(0, 0, 0);
	this->torque = 0.0f;
	this->density = 1.0f;
	this->angularVelocity = 0.0f;
	this->dynamicFriction = 0.0f;
	this->impulse = 0.0f;
	this->inertia = 0.0f;
	this->inv_intertia = 0.0f;
	this->inv_mass = 0.0f;
	this->mass = 0.0f;
	this->velocity = glm::vec3(0, 0, 0);
	this->staticFriction = 0.0f;
	this->restitution = restitution;
	this->setParent(_parent);
	if (IsStatic)
		SetStatic();
}

bRigidbody2D::bRigidbody2D(float restitution, bool IsStatic)
{
	this->force = glm::vec3(0, 0, 0);
	this->torque = 0.0f;
	this->density = 1.0f;
	this->angularVelocity = 0.0f;
	this->dynamicFriction = 0.0f;
	this->impulse = 0.0f;
	this->inertia = 0.0f;
	this->inv_intertia = 0.0f;
	this->inv_mass = 0.0f;
	this->mass = 0.0f;
	this->velocity = glm::vec3(0, 0, 0);
	this->staticFriction = 0.0f;
	this->restitution = restitution;
	if (IsStatic)
		SetStatic();
}

void bRigidbody2D::ApplyImpulse(const glm::vec3& impulse, const glm::vec3& contactVector)
{
	//A->ApplyImpulse(-(impulse), (B->getParent()->myTransform->position - A->getParent()->myTransform->position));
	//B->ApplyImpulse(impulse, (A->getParent()->myTransform->position - B->getParent()->myTransform->position));
	
	velocity += this->inv_mass * impulse;
	angularVelocity += inv_intertia * (contactVector.x * impulse.y - contactVector.y * impulse.x); //Cross(contactVector, impulse);
}

void bRigidbody2D::SetStatic()
{
	inertia = 0.0f;
	inv_intertia = 0.0f;
	mass = 0.0f;
	inv_mass = 0.0f;
}