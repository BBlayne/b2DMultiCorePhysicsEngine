#include "bCircle.h"

bCircle::bCircle()
{
	float _radius = 10;
	float density = 1.0f;

	setRadius(_radius);


	body->mass = 3.14159f * std::pow(getRadius(), 2) * density;
	body->inv_mass = (body->mass) ? 1.0f / body->mass : 0.0f;
	body->inertia = body->mass * std::pow(getRadius(), 2);
	body->inv_intertia = (body->inertia) ? 1.0f / body->inertia : 0.0f;
}

bCircle::bCircle(bRigidbody2D* _body, glm::vec3 _pos, float _radius, float density)
{
	setRadius(_radius);
	this->body = _body;
	body->mass = 3.14159f * std::pow(_radius, 2) * density;
	body->inv_mass = (body->mass) ? 1.0f / body->mass : 0.0f;
	body->inertia = body->mass * std::pow(_radius, 2);
	body->inv_intertia = (body->inertia) ? 1.0f / body->inertia : 0.0f;
}

bCircle::Type bCircle::GetType()
{
	return Type::circle;
}

void bCircle::Init()
{
	ComputeMass(1.0f);
}

void bCircle::ComputeMass(float density)
{
	body->mass = 3.14159f * std::pow(radius,2) * density;
	body->inv_mass = (body->mass) ? 1.0f / body->mass : 0.0f;
	body->inertia = body->mass * std::pow(radius,2);
	body->inv_intertia = (body->inertia) ? 1.0f / body->inertia : 0.0f;
}

void bCircle::SetOrient(float radians)
{

}

void bCircle::UpdateExtents(float radians)
{

}

glm::vec2 bCircle::GetExtents()
{
	return glm::vec2(radius*2, radius*2);
}
