#include "bRectangle.h"

bPoly::bPoly()
{
	width = 0;
	height = 0;
}

bPoly::bPoly(float density, std::vector<glm::vec3> vertices)
{
	width = 0;
	height = 0;
	ComputeMass(density, vertices); //calls compute mass.
}

bPoly::bPoly(bRigidbody2D* _body, glm::vec3 _pos, float _scaleWidth, float _scaleHeight, float density, std::vector<glm::vec3> vertices)
{
	Init(); //calls compute mass.
	this->body = _body;
	width = (_scaleWidth * UNIT_SCALE * 2.5f);
	height = (_scaleHeight * UNIT_SCALE * 2.5f);
	setAABB(_pos, width, height);
	
}

void bPoly::setAABB(glm::vec3 position, float width, float height)
{
	min = position + glm::vec3((-width / 2), (-height / 2), 0);
	max = position + glm::vec3((width / 2), (height / 2), 0);
}

void bPoly::SetOrient(float orient)
{
	this->orientation = glm::rotate(glm::mat4(1.0f), orient, glm::vec3(0, 0, 1));
}

bPoly::Type bPoly::GetType()
{
	return Type::poly;
}


void bPoly::UpdateExtents(float scale)
{
	this->max *= scale;
	this->min *= scale;
}

glm::vec2 bPoly::GetExtents()
{
	return glm::vec2(width, height);
}

void bPoly::ComputeMass(float density, std::vector<glm::vec3> vertices)
{
	glm::vec3 centroid = glm::vec3(0, 0, 0);
	float area = 1.0f;
	float I = 0;
	const float k_inv3 = 1.0f / 3.0f;

	for (int i = 0; i < vertices.size(); i++)
	{

		glm::vec3 p1(vertices.at(i));
		int i2 = i + 1 < vertices.size() ? i + 1 : 0;
		glm::vec3 p2(vertices.at(i2));

		float mDensity = p1.x * p2.y - p1.y * p2.x; // 2D cross product
		float triangle_area = 0.5f * mDensity;

		area += triangle_area;

		centroid += triangle_area * k_inv3 * (p1 + p2);
		float intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
		float inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
		I = (0.25f * k_inv3 * mDensity) * (intx2 + inty2);

	}

	centroid *= 1.0f / area;

	body->mass = density * area;
	body->inv_mass = (body->mass) ? 1.0f / body->mass : 0.0f;
	body->inertia = I * density;
	body->inv_intertia = body->inertia ? 1.0f / body->inertia : 0.0f;
}

void bPoly::Init()
{
	//Compute Mass
}
