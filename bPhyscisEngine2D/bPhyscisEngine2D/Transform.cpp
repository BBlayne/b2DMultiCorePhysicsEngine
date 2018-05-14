#include "Transform.h"
#include "bRigidBody2D.h"
#include "Shape.h"

void Transform::SetOrient(float angle_in_rads)
{
	orientation_in_radians = angle_in_rads;
}

Transform::Transform()
{
	this->translation = glm::mat4(1.0f);
	this->rotation = glm::mat4(1.0f);
	this->scale = glm::scale(glm::vec3(UNIT_SCALE, UNIT_SCALE, 1.0f));
	this->SetOrient(0);
}

Transform::Transform(float orientation_radians)
{
	this->translation = glm::mat4(1.0f);
	this->rotation = glm::mat4(1.0f);
	this->scale = glm::scale(glm::vec3(UNIT_SCALE, UNIT_SCALE, 1.0f));
	this->SetOrient(orientation_radians);
}

Transform::Transform(float orientation_radians, glm::vec3 pos)
{
	this->translation = glm::mat4(1.0f);
	this->rotation = glm::mat4(1.0f);
	this->scale = glm::scale(glm::vec3(UNIT_SCALE, UNIT_SCALE, 1.0f));
	this->position = pos;
	this->SetOrient(orientation_radians);
}

void Transform::setScale(glm::vec3 _scale)
{
	this->scale = this->scale * glm::scale(glm::vec3(_scale.x, _scale.y, _scale.z));
}