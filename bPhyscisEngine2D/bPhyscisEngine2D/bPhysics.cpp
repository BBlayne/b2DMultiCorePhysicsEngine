#include "bPhysics.h"
#include <time.h>
#include <ctime>
#include "Dependencies\glfw3.h"
#include <iostream>
#include <algorithm>
#include <string>

bPhysics::bPhysics() {
	accumulator = 0;
	bVec3 temp;
	temp.x = 0.0f;
	temp.y = 9.80665f * gravityScale;
	temp.z = 0.0f;
	uiTargetDevice = 0;
	bGravity = temp;
	// In units seconds
	frameStart = glfwGetTime();
	temp_contacts = (manifold_cl*)malloc(sizeof(manifold_cl));
	_contacts = (manifold_cl*)malloc(sizeof(manifold_cl));
}

void bPhysics::IntegrateForce(bRigidbody2D *b, float dt)
{
	try
	{
		if (b->inv_mass == 0.0f)
		{

		}
		else
		{
			b->velocity += (b->force * b->impulse - gravity) * (dt / 2.0f);
			b->angularVelocity += b->torque * b->inv_intertia * (dt / 2.0f);
		}
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception: IntegrateForce: " << e.what() << std::endl;
	}
}

void bPhysics::PositionalCorrection(bManifold* m)
{
	try
	{
		const float k_slop = 0.05f; // Penetration allowance
		const float percent = 0.4f; // Penetration percentage to correct
		glm::vec3 correction = (std::max(m->penetration - k_slop, 0.0f) / (m->A->inv_mass + m->B->inv_mass)) * m->normal * percent;

		m->A->getParent()->myTransform->position -= correction * m->A->inv_mass;
		m->A->transform->position -= correction * m->A->inv_mass;

		m->B->getParent()->myTransform->position += correction * m->B->inv_mass;
		m->B->transform->position += correction * m->B->inv_mass;
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception is here (PositionalCorrection): " << e.what() << std::endl;
	}
}

void bPhysics::IntegrateForces(bRigidbody2D *b, float dt)
{
	try
	{
		if (b->inv_mass == 0.0f)
			return;

		b->velocity += (b->force * b->impulse - gravity) * (dt / 2.0f);
		b->angularVelocity += b->torque * b->inv_intertia * (dt / 2.0f);
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception is here (IntegrateForces): " << e.what() << std::endl;
	}
}

void bPhysics::IntegrateVelocity(bRigidbody2D *b, float dt)
{
	try
	{
		if (b->inv_mass == 0.0f)
			return;

		b->getParent()->myTransform->position += b->velocity * (dt);
		b->getParent()->myTransform->orientation_in_radians += b->angularVelocity * dt;
		b->getParent()->myTransform->SetOrient(b->getParent()->myTransform->orientation_in_radians);
		IntegrateForces(b, dt);
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception is here (IntegrateVelocity): " << e.what() << std::endl;
	}
}

void bPhysics::Step() 
{
	contacts.clear();
	if (tree != nullptr)
	{
		GeneratePairsQuadTree();
	}
	else
	{
		std::cout << "Error: No Tree detected..." << std::endl;
		for (int i = 0; i < bodies.size(); i++)
		{
			bRigidbody2D* A = bodies[i];

			for (int j = 0; j < bodies.size(); ++j)
			{
				bRigidbody2D* B = bodies[j];
				if (A->inv_mass == 0 && B->inv_mass == 0)
				{

				}
				else
				{
					bManifold m(A, B);
					m.Solve();
					if (m.isColliding)
					{
						contacts.emplace_back(m);
					}
				}				
			}
		}
	}

	this->ComputePhysics();
}

void bPhysics::Step_CL(int numWidgets, widget*& _widgets)
{
	int numContacts = 0;
	cl_int maxPairsSize = numWidgets * numWidgets;
	contacts.clear();
	resizeBuffers_CL(numWidgets);
	_contacts = (manifold_cl*)malloc(maxPairsSize * sizeof(manifold_cl));
	for (int i = 0; i < maxPairsSize; i++)
	{
		_contacts[i] = d_contacts[i];
	}

	updateBuffers_CL(_widgets, numWidgets, numContacts, _contacts);
	
	runGeneratePairsCL(_widgets, numWidgets, numContacts, _contacts);


	updateCPUBuffers_CL(_widgets, _contacts, numWidgets, numContacts);
	//this->ComputePhysics_CL(numWidgets, numContacts, _widgets, _contacts);
	this->runComputePhysics_CL(numWidgets, numContacts, _widgets, _contacts);
	ret = clReleaseMemObject(memobj_widgets);
	ret = clReleaseMemObject(memobj_contacts);
	ret = clReleaseMemObject(memobj_contacts_size);
	ret = clReleaseMemObject(memobj_widgets_size);
	ret = clReleaseMemObject(cpu_memobj_widgets);
	ret = clReleaseMemObject(cpu_memobj_contacts);
	ret = clReleaseMemObject(cpu_memobj_contacts_size);
	ret = clReleaseMemObject(cpu_memobj_widgets_size);
	ret = clReleaseMemObject(cpu_memobj_gravity);
	ret = clReleaseMemObject(cpu_memobj_dt);
	free(_contacts);
}

float bDot(bVec3 _a, bVec3 _b)
{
	return (_a.x * _b.x + _a.y * _b.y + _a.z * _b.z);
}

bVec3 bMult(bVec3 _a, float _b)
{
	bVec3 temp;
	temp.x = _a.x * _b;
	temp.y = _a.y * _b;
	temp.z = _a.z * _b;
	return temp;
}

bVec3 bSum(bVec3 _a, bVec3 _b)
{
	bVec3 temp;
	temp.x = _a.x + _b.x;
	temp.y = _a.y + _b.y;
	temp.z = _a.z + _b.z;
	return temp;
}

bVec3 bDiff(bVec3 _a, bVec3 _b)
{
	bVec3 temp;
	temp.x = _a.x - _b.x;
	temp.y = _a.y - _b.y;
	temp.z = _a.z - _b.z;
	return temp;
}

bVec3 bDiv(bVec3 _a, float _b)
{
	bVec3 temp;
	temp.x = _a.x / _b;
	temp.y = _a.y / _b;
	temp.z = _a.z / _b;
	return temp;
}

bool isEqual(bVec3 _a, bVec3 _b)
{
	if (_a.x == _b.x && _a.y == _b.y && _a.z == _b.z)
	{
		return true;
	}
	else
	{
		return false;
	}
}

float bMax(float _a, float _b)
{
	// presumably if _a == _b then it doesn't matter which is returned.
	if (_a > _b)
	{
		return _a;
	}
	else
	{
		return _b;
	}
}

int bExp(int x, int y)
{
	if (y < 0) {
		switch (x) {
		case 0:
			return INTMAX_MAX;
		case 1:
			return 1;
		case -1:
			return y % 2 ? -1 : 1;
		}
		return 0;
	}
	intmax_t z = 1;
	intmax_t base = x;
	for (;;) {
		if (y & 1) {
			z *= base;
		}
		y >>= 1;
		if (y == 0) {
			break;
		}
		base *= base;
	}
	return z;
}

float bExp(float _base)
{
	return _base * _base;
}


float bLength2(bVec3 _a)
{
	return bExp(_a.x) + bExp(_a.y) + bExp(_a.z);
}

void bPhysics::ComputePhysics_CL(int numWidgets, int& numContacts, widget*& _widgets, manifold_cl*& _contacts)
{

	for (int i = 0; i < numWidgets; i++)
	{
		if (_widgets[i].inv_mass_cl == 0.0f)
			continue;

		_widgets[i].velocity_cl = bSum(bMult(bDiff(bMult(_widgets[i].force_cl, _widgets[i].impulse_cl), bGravity), (dt / 2.0f)), _widgets[i].velocity_cl);
		_widgets[i].angularVelocity_cl += _widgets[i].torque_cl * _widgets[i].inv_intertia_cl * (dt / 2.0f);
	}

	// init collision
	for (int i = 0; i < numContacts; i++)
	{
		// get lowest restitution
		_contacts[i].e_cl = glm::min(_widgets[_contacts[i].A_cl].restitution_cl, _widgets[_contacts[i].B_cl].restitution_cl);
		// Calculate static and dynamic friction
		_contacts[i].sf_cl = std::sqrt(_widgets[_contacts[i].A_cl].staticFriction_cl * _widgets[_contacts[i].B_cl].staticFriction_cl);
		_contacts[i].df_cl = std::sqrt(_widgets[_contacts[i].A_cl].dynamicFriction_cl * _widgets[_contacts[i].B_cl].dynamicFriction_cl);
	}


	// Solve collisions
	//for (int j = 0; j < m_iterations; j++)
	for (int i = 0; i < numContacts; i++)
	{
		bVec3 rv;
		bVec3 temp = bDiff(_widgets[_contacts[i].B_cl].velocity_cl, _widgets[_contacts[i].A_cl].velocity_cl);
		rv.x = temp.x;
		rv.y = temp.y;
		rv.z = temp.z;

		float contactVel = bDot(rv, _contacts[i].normal_cl);
		if (contactVel > 0)
			continue;

		float e = glm::min(_widgets[_contacts[i].A_cl].restitution_cl, _widgets[_contacts[i].B_cl].restitution_cl);

		float numerator = -(1.0f + e) * contactVel;
		float j = numerator / (_widgets[_contacts[i].A_cl].inv_mass_cl + _widgets[_contacts[i].B_cl].inv_mass_cl);
		bVec3 impulse = bMult(_contacts[i].normal_cl, j);//  *1.0f;

		bVec3 negative_impulse;
		negative_impulse.x = -impulse.x;
		negative_impulse.y = -impulse.y;
		negative_impulse.z = -impulse.z;

		_widgets[_contacts[i].A_cl].velocity_cl = bSum(bMult(negative_impulse, _widgets[_contacts[i].A_cl].inv_mass_cl), _widgets[_contacts[i].A_cl].velocity_cl);
		bVec3 contactVector = bDiff(_widgets[_contacts[i].B_cl].t_cl.position_cl, _widgets[_contacts[i].A_cl].t_cl.position_cl);
		_widgets[_contacts[i].A_cl].angularVelocity_cl += _widgets[_contacts[i].A_cl].inv_intertia_cl * (contactVector.x * impulse.y - contactVector.y * impulse.x);

		_widgets[_contacts[i].B_cl].velocity_cl = bSum(bMult(impulse, _widgets[_contacts[i].B_cl].inv_mass_cl), _widgets[_contacts[i].B_cl].velocity_cl);
		bVec3 contactVector2 = bDiff(_widgets[_contacts[i].A_cl].t_cl.position_cl, _widgets[_contacts[i].B_cl].t_cl.position_cl);
		_widgets[_contacts[i].B_cl].angularVelocity_cl += _widgets[_contacts[i].B_cl].inv_intertia_cl * (contactVector2.x * impulse.y - contactVector2.y * impulse.x);

	}

	// integrate velocities
	for (int i = 0; i < numWidgets; i++)
	{
		//this->IntegrateVelocity(this->bodies.at(i), dt);
		if (_widgets[i].inv_mass_cl == 0.0f)
			continue;

		_widgets[i].t_cl.position_cl = bSum(bMult(_widgets[i].velocity_cl, (dt)), _widgets[i].t_cl.position_cl);
		_widgets[i].t_cl.orientation_in_radians_cl += _widgets[i].angularVelocity_cl * dt;

		if (_widgets[i].inv_mass_cl == 0.0f)
			continue;

		_widgets[i].velocity_cl = bSum(bMult(bDiff(bMult(_widgets[i].force_cl, _widgets[i].impulse_cl), bGravity), (dt / 2.0f)), _widgets[i].velocity_cl);
		_widgets[i].angularVelocity_cl += _widgets[i].torque_cl * _widgets[i].inv_intertia_cl * (dt / 2.0f);


	}

	// correct positions
	for (int i = 0; i < numContacts; i++)
	{
		//PositionalCorrection(&contacts.at(i));
		const float k_slop = 0.05f; // Penetration allowance
		const float percent = 0.4f; // Penetration percentage to correct

		float tempMax = bMax(_contacts[i].penetration_cl - k_slop, 0.0f);
		float tempVal = (_widgets[_contacts[i].A_cl].inv_mass_cl + _widgets[_contacts[i].B_cl].inv_mass_cl);
		tempVal = tempMax / tempVal;

		bVec3 temp3 = bMult(_contacts[i].normal_cl, tempVal);
		bVec3 correction = bMult(temp3, percent);

		_widgets[_contacts[i].A_cl].t_cl.position_cl = bDiff(_widgets[_contacts[i].A_cl].t_cl.position_cl, bMult(correction, _widgets[_contacts[i].A_cl].inv_mass_cl));
		_widgets[_contacts[i].B_cl].t_cl.position_cl = bSum(bMult(correction, _widgets[_contacts[i].B_cl].inv_mass_cl), _widgets[_contacts[i].B_cl].t_cl.position_cl);

	}

	// clear all forces
	for (int i = 0; i < numWidgets; i++)
	{
		bVec3 temp_clear_all_forces;
		temp_clear_all_forces.x = 0;
		temp_clear_all_forces.y = 0;
		temp_clear_all_forces.z = 0;
		_widgets[i].force_cl = temp_clear_all_forces;
	}
}

void bPhysics::ComputePhysics()
{
	for (int i = 0; i < bodies.size(); i++)
		this->IntegrateForces(this->bodies.at(i), dt);

	// init collision
	for (int i = 0; i < contacts.size(); i++)
	{
		contacts.at(i).Initialize();
	}


	// Solve collisions
	//for (int j = 0; j < m_iterations; j++)
	for (int i = 0; i < contacts.size(); i++)
		contacts.at(i).ApplyImpulseSimplified();

	// integrate velocities
	for (int i = 0; i < bodies.size(); i++)
		this->IntegrateVelocity(this->bodies.at(i), dt);

	// correct positions
	for (int i = 0; i < contacts.size(); i++)
		PositionalCorrection(&contacts.at(i));

	// clear all forces
	for (int i = 0; i < bodies.size(); i++)
	{
		bRigidbody2D *temp = bodies.at(i);
		//temp->impulse = 0;
		temp->force = glm::vec3(0, 0, 0);
	}
}

bRigidbody2D* bPhysics::TryGetBody(int index)
{
	try {
		bRigidbody2D *temp;
		temp = bodies.at(index);
		return temp;
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception@: TryGetBody. " << e.what() << std::endl;
	}
	return NULL;
}

void bPhysics::GeneratePairsQuadTree()
{

	std::vector<bRigidbody2D*> possibleContacts;

	for (int i = 4; i < bodies.size(); i++)
	{
		possibleContacts.clear();
		bRigidbody2D* A = bodies.at(i);
		this->tree->retrieve(possibleContacts, A->getParent());
		for (int j = 0; j < possibleContacts.size(); ++j)
		{
			bRigidbody2D* B = possibleContacts.at(j);
			if ((A->inv_mass == 0 && B->inv_mass == 0) || 
				B->getParent()->getWidgetID() == A->getParent()->getWidgetID())
			{

			}
			else
			{
				bManifold m(A, B);
				m.Solve();
				if (m.isColliding)
				{
					contacts.emplace_back(m);
				}
			}

		}
	}
}

void bPhysics::GeneratePairsUnthreaded()
{
	for (int i = 0; i < bodies.size(); i++)
	{
		bRigidbody2D* A = bodies.at(i);
		for (int j = i + 1; j < bodies.size(); ++j)
		{
			bRigidbody2D* B = bodies.at(j);
			if (A->inv_mass == 0 && B->inv_mass == 0)
				continue;

			bManifold m(A, B);
			m.Solve();
			if (m.isColliding)
			{
				contacts.emplace_back(m);
			}

		}
	}
}

void bPhysics::GeneratePairs_CL(manifold_cl*& _contacts, widget*& _widgets, int _WidgetSize, int& _contactsSize)
{
	
	//__kernel void generatePairsKernel__global
	for (int i = 0; i < _WidgetSize; i++)
	{
		widget temp_widget_a_cl = _widgets[i];
		//bRigidbody2D* A = bodies.at(i);
		for (int j = 0; j < _WidgetSize; j++)
		{
			widget temp_widget_b_cl = _widgets[j];
			//bRigidbody2D* B = bodies.at(j);
			if (temp_widget_a_cl.inv_mass_cl == 0 && temp_widget_b_cl.inv_mass_cl == 0)
				continue;

			if (temp_widget_a_cl.id_cl == temp_widget_b_cl.id_cl)
				continue;

			manifold_cl _m;
			_m.isColliding_cl = false;
			bVec3 temp;
			temp.x = 0;
			temp.y = 0;
			temp.z = 0;
			_m.normal_cl = temp;
			_m.penetration_cl = 0;
			_m.A_cl = i;
			_m.B_cl = j;

			//bManifold m(A, B);
			//m.Solve();
			if (temp_widget_a_cl.shape_cl == 0)
			{
				// polygons don't collide with other objects
			}
			else
			{
				if (temp_widget_b_cl.shape_cl == 1)
				{
					// CircleCircle
					bVec3 _normal = bDiff(temp_widget_b_cl.t_cl.position_cl, temp_widget_a_cl.t_cl.position_cl);

					float r = (temp_widget_a_cl.width_cl) + (temp_widget_b_cl.width_cl);
					float temp = bLength2(_normal);
					//float temp2 = glm::length2(glm::vec3(_normal.x, _normal.y, 0));
					//std::cout << "temp: " << temp << "r" << r << std::endl;
					if (temp > bExp(r))
					{
						// does not contact
						_m.isColliding_cl = false;
						continue;
					}

					// Circles have collided, compute manifold.
					float distance = sqrt(bLength2(_normal));

					//std::cout << "Circle collission" << std::endl;
					// If distance between circles is not zero
					if (distance != 0)
					{
						// Penetration Distance is difference between radius and distance
						_m.penetration_cl = r - distance;
						//m->contact_points.push_back(A->body->getParent()->myTransform->position);
						//m->contact_points.push_back(B->body->getParent()->myTransform->position);
						// Utilize our distance since we performed sqrt on it already within Length( )
						// Points from A to B, and is a unit vector
						_m.normal_cl = bDiv(_normal, distance);
						_m.isColliding_cl = true;
					}
					else // circles are on the same position (What does this mean?)
					{
						//m->contact_points.push_back(A->body->getParent()->myTransform->position);
						//m->contact_points.push_back(B->body->getParent()->myTransform->position);
						_m.penetration_cl = temp_widget_a_cl.width_cl / 2;
						bVec3 _temp;
						_temp.x = 1.0f;
						_temp.y = 0;
						_temp.z = 0;
						_m.normal_cl = _temp;
						//m->contact_count = 1;
						_m.isColliding_cl = true;
					}
				}
				else
				{
					// CirclePoly
					// Vector from A to B
					bVec3 _normal = bDiff(temp_widget_b_cl.t_cl.position_cl, temp_widget_a_cl.t_cl.position_cl);

					// Closest point on A to center of B
					bVec3 closest = _normal;
					bVec3 _temp_min;
					_temp_min.x = (-temp_widget_b_cl.width_cl / 2);
					_temp_min.y = (-temp_widget_b_cl.height_cl / 2);
					_temp_min.z = 0;

					bVec3 min = bSum(temp_widget_b_cl.t_cl.position_cl,_temp_min);

					bVec3 _temp_max;
					_temp_max.x = (temp_widget_b_cl.width_cl / 2);
					_temp_max.y = (temp_widget_b_cl.height_cl / 2);
					_temp_max.z = 0;
					bVec3 max = bSum(temp_widget_b_cl.t_cl.position_cl, _temp_max);

					//bVec3 max = temp_widget_b_cl.t_cl.position +
					//	glm::vec3((temp_widget_b_cl.width / 2), (temp_widget_b_cl.height / 2), 0);

					// Calculate half extents along each axis
					float x_extent = (max.x - min.x) / 2;
					float y_extent = (max.y - min.y) / 2;

					// Clamp point to edges of the AABB
					closest.x = glm::clamp(closest.x, -x_extent, x_extent);
					closest.y = glm::clamp(closest.y, -y_extent, y_extent);

					bool inside = false;

					// Circle is inside the AABB, so we need to clamp the circle's center
					// to the closest edge
					if (isEqual(_normal, closest))
					{
						inside = true;

						// Find closest axis
						if (glm::abs(_normal.x) > glm::abs(_normal.y))
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

					bVec3 normalizedNormal = bDiff(_normal, closest);
					float distance = bLength2(normalizedNormal);

					float radius = temp_widget_a_cl.width_cl;

					// Early out of the radius is shorter than distance to closest point and
					// Circle not inside the AABB
					if ((distance > std::pow(radius, 2)) && !inside)
						continue;

					// Avoided sqrt until we needed
					distance = std::sqrt(distance);
					bVec3 negativeNormalizedNormal;
					negativeNormalizedNormal.x = -normalizedNormal.x;
					negativeNormalizedNormal.y = -normalizedNormal.y;
					negativeNormalizedNormal.z = -normalizedNormal.z;
					// Collision normal needs to be flipped to point outside if circle was
					// inside the AABB
					if (inside)
					{
						//m->contact_points.push_back(A->body->getParent()->myTransform->position);
						//m->contact_points.push_back(B->body->getParent()->myTransform->position);

						_m.normal_cl = bDiv(negativeNormalizedNormal, distance);
						_m.penetration_cl = radius - distance;
					}
					else
					{
						//m->contact_points.push_back(A->body->getParent()->myTransform->position);
						//m->contact_points.push_back(B->body->getParent()->myTransform->position);
						_m.normal_cl = bDiv(normalizedNormal, distance);
						_m.penetration_cl = radius - distance;
					}
					//m->contact_count = 1;
					_m.isColliding_cl = true;
				}
			}

			if (_m.isColliding_cl)
			{
				//contacts.emplace_back(m);
				// new array of contacts
				_contacts = (manifold_cl*)realloc(_contacts, (_contactsSize + 1) * sizeof(manifold_cl));

				_contacts[_contactsSize] = _m;
				_contactsSize++;

				//contacts.emplace_back(m);
			}
		}
	}
}

void bPhysics::FixedUpdate()
{
	const float currentTime = glfwGetTime();

	double frameTime = currentTime - frameStart;

	// Store the time elapsed since the last frame began
	accumulator += frameTime;

	// Record the starting of this frame
	frameStart = currentTime;

	// Avoid spiral of death and clamp dt, thus clamping
	// how many times the UpdatePhysics can be called in
	// a single game loop.
	if (accumulator > 0.2f)
		accumulator = 0.2f;

	while (accumulator >= dt)
	{
		Step();
		accumulator -= dt;
	}

	const float alpha = accumulator / dt;
}

void bPhysics::FixedUpdate_CL(int _numWidgets, widget*& _widgets)
{
	const float currentTime = glfwGetTime();

	double frameTime = currentTime - frameStart;

	// Store the time elapsed since the last frame began
	accumulator += frameTime;

	// Record the starting of this frame
	frameStart = currentTime;

	// Avoid spiral of death and clamp dt, thus clamping
	// how many times the UpdatePhysics can be called in
	// a single game loop.
	if (accumulator > 0.2f)
		accumulator = 0.2f;

	start = std::chrono::high_resolution_clock::now();
	while (accumulator >= dt)
	{
		Step_CL(_numWidgets, _widgets);
		accumulator -= dt;
	}
	end = std::chrono::high_resolution_clock::now();
	this->duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	sprintf_s(physicsTimer, "FixedUpdate: %d", (int)this->duration);

	const float alpha = accumulator / dt;
}

void bPhysics::clLogging()
{
	int i, j;

	char* info;
	size_t infoSize;
	cl_uint platformCount;
	cl_platform_id *platforms;
	const char* attributeNames[5] = 
	{ 
		"Name", 
		"Vendor",
		"Version", 
		"Profile", 
		"Extensions" 
	};

	const cl_platform_info attributeTypes[5] = 
	{ 
		CL_PLATFORM_NAME, 
		CL_PLATFORM_VENDOR,
		CL_PLATFORM_VERSION, 
		CL_PLATFORM_PROFILE, 
		CL_PLATFORM_EXTENSIONS 
	};

	const int attributeCount = sizeof(attributeNames) / sizeof(char*);

	// get platform count
	clGetPlatformIDs(5, NULL, &platformCount);

	// get all platforms
	platforms = (cl_platform_id*)malloc(sizeof(cl_platform_id) * platformCount);
	clGetPlatformIDs(platformCount, platforms, NULL);

	// for each platform print all attributes
	for (i = 0; i < platformCount; i++) {

		printf(" %d. Platform ", i + 1);
		std::cout << std::endl;
		for (j = 0; j < attributeCount; j++) {

			// get platform attribute value size
			clGetPlatformInfo(platforms[i], attributeTypes[j], 0, NULL, &infoSize);
			info = (char*)malloc(infoSize);

			// get platform attribute value
			clGetPlatformInfo(platforms[i], attributeTypes[j], infoSize, info, NULL);

			printf("  %d.%d %-11s: %s", i + 1, j + 1, attributeNames[j], info);
			std::cout << std::endl;
			free(info);

		}
		//printf("n");

	}
	std::cout << "Press ENTER to continue...";
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	free(platforms);
}

bool bPhysics::bCL_Init(widget*& _widgets, int sizeWidgets)
{
	char* info;
	size_t infoSize;
	const char* attributeNames[5] =
	{
		"Name",
		"Vendor",
		"Version",
		"Profile",
		"Extensions"
	};

	const cl_platform_info attributeTypes[5] =
	{
		CL_PLATFORM_NAME,
		CL_PLATFORM_VENDOR,
		CL_PLATFORM_VERSION,
		CL_PLATFORM_PROFILE,
		CL_PLATFORM_EXTENSIONS
	};
	const int platformAttributeCount = sizeof(attributeNames) / sizeof(char*);
	// get platform count
	ret = clGetPlatformIDs(5, NULL, &platformCount);
	// get all platforms
	platforms = (cl_platform_id*)malloc(sizeof(cl_platform_id) * platformCount);
	ret = clGetPlatformIDs(platformCount, platforms, NULL);
	// for each platform print all attributes
	for (int i = 0; i < platformCount; i++) {
		printf(" %d. Platform ", i + 1);
		std::cout << std::endl;
		for (int j = 0; j < platformAttributeCount; j++) {

			// get platform attribute value size
			clGetPlatformInfo(platforms[i], attributeTypes[j], 0, NULL, &infoSize);
			info = (char*)malloc(infoSize);

			// get platform attribute value
			clGetPlatformInfo(platforms[i], attributeTypes[j], infoSize, info, NULL);

			printf("  %d.%d %-11s: %s", i + 1, j + 1, attributeNames[j], info);
			std::cout << std::endl;
			free(info);
		}
	}
	infoSize = 0;
	cl_uint addressBits;
	cl_ulong globalMemory;
	cl_bool isCompilerAvailable;
	cl_bool isDeviceAvailable;
	/* Device attributes */
	cl_uint* deviceCount = (cl_uint*)malloc(platformCount*sizeof(cl_uint));
	cl_device_id** devices = (cl_device_id**)malloc(platformCount*sizeof(cl_device_id*));

	cl_uint* CPUDeviceCount = (cl_uint*)malloc(platformCount*sizeof(cl_uint));
	cl_device_id** CPUDevices = (cl_device_id**)malloc(platformCount*sizeof(cl_device_id*));

	const char* deviceAttributeNames[8] =
	{
		"Name",
		"Global Memory",
		"Address Width",
		"Max Clock Frequency",
		"Compute Units",
		"Extentions",
		"Status",
		"Compiler Status"
	};

	const cl_platform_info deviceAttributeTypes[8] =
	{
		CL_DEVICE_NAME,
		CL_DEVICE_GLOBAL_MEM_SIZE,
		CL_DEVICE_ADDRESS_BITS,
		CL_DEVICE_MAX_CLOCK_FREQUENCY,
		CL_DEVICE_MAX_COMPUTE_UNITS,
		CL_DEVICE_EXTENSIONS,
		CL_DEVICE_AVAILABLE,
		CL_DEVICE_COMPILER_AVAILABLE
	};

	const int deviceAttributeCount = sizeof(deviceAttributeNames) / sizeof(char*);
	int maxFrequency;
	/* Find number of devices & their information */
	int temp = 0;
	// get platform count
	std::cout << std::endl;
	std::cout << "Your available devices across all platforms..." << std::endl;
	for (int k = 0; k < platformCount; k++)
	{
		printf(" %d. Platform ", k + 1);
		std::cout << std::endl;
		ret = clGetDeviceIDs(platforms[k], CL_DEVICE_TYPE_ALL, 1, NULL, &deviceCount[k]);
		ret = clGetDeviceIDs(platforms[k], CL_DEVICE_TYPE_CPU, 1, NULL, &CPUDeviceCount[k]);

		// get all platforms
		if (CPUDeviceCount[k] == 0)
		{
			ret = clGetDeviceIDs(platforms[k], CL_DEVICE_TYPE_ALL, 1, NULL, &CPUDeviceCount[k]);
			CPUDevices[k] = (cl_device_id*)malloc(sizeof(cl_device_id) * CPUDeviceCount[k]);
			ret = clGetDeviceIDs(platforms[k], CL_DEVICE_TYPE_ALL, CPUDeviceCount[k], CPUDevices[k], NULL);
		}
		else
		{
			CPUDevices[k] = (cl_device_id*)malloc(sizeof(cl_device_id) * CPUDeviceCount[k]);
			ret = clGetDeviceIDs(platforms[k], CL_DEVICE_TYPE_CPU, CPUDeviceCount[k], CPUDevices[k], NULL);
		}
		devices[k] = (cl_device_id*)malloc(sizeof(cl_device_id) * deviceCount[k]);		
		ret = clGetDeviceIDs(platforms[k], CL_DEVICE_TYPE_ALL, deviceCount[k], devices[k], NULL);
		
		// for each platform print all attributes
		for (int i = 0; i < deviceCount[k]; i++) {
			printf("  %d. Device ", i + 1);
			std::cout << std::endl;
			for (int j = 0; j < deviceAttributeCount; j++) {				
				switch (j)
				{
				case 0:
					// get platform attribute value size
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], 0, NULL, &infoSize);
					info = (char*)malloc(infoSize);

					// get platform attribute value
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], infoSize, info, NULL);
					printf("   %d.%d %-11s: %s", i + 1, j + 1, deviceAttributeNames[j], info);
					free(info);
					break;
				case 1:
					// get platform attribute value size
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], 0, NULL, &infoSize);
					// get platform attribute value
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], infoSize, &globalMemory, NULL);
					// divide by 1M to get value in Megabytes
					globalMemory /= 1000000;
					printf("   %d.%d %-11s: %u", i + 1, j + 1, deviceAttributeNames[j], globalMemory);
					break;
				case 2:
					// get platform attribute value size
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], 0, NULL, &infoSize);
					// get platform attribute value
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], infoSize, &addressBits, NULL);
					printf("   %d.%d %-11s: %d", i + 1, j + 1, deviceAttributeNames[j], addressBits);
					break;
				case 3:
					// get platform attribute value size
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], 0, NULL, &infoSize);
					// get platform attribute value
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], infoSize, &maxFrequency, NULL);
					printf("   %d.%d %-11s: %d", i + 1, j + 1, deviceAttributeNames[j], maxFrequency);
					break;
				case 4:
					// get platform attribute value size
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], 0, NULL, &infoSize);
					// get platform attribute value
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], infoSize, &uiNumComputeUnits, NULL);
					printf("   %d.%d %-11s: %d", i + 1, j + 1, deviceAttributeNames[j], uiNumComputeUnits);
					break;
				case 5:
					// get platform attribute value size
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], 0, NULL, &infoSize);
					info = (char*)malloc(infoSize);
					// get platform attribute value
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], infoSize, info, NULL);
					printf("   %d.%d %-11s: %s", i + 1, j + 1, deviceAttributeNames[j], info);
					free(info);
					break;
				case 6:
					temp = 0;
					// get platform attribute value size
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], 0, NULL, &infoSize);
					// get platform attribute value
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], infoSize, &isDeviceAvailable, NULL);
					if (isDeviceAvailable)
						temp = 1;

					printf("   %d.%d %-11s: %d", i + 1, j + 1, deviceAttributeNames[j], temp);
					break;
				case 7:
					temp = 0;
					// get platform attribute value size
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], 0, NULL, &infoSize);
					// get platform attribute value
					clGetDeviceInfo(devices[k][i], deviceAttributeTypes[j], infoSize, &isCompilerAvailable, NULL);
					if (isCompilerAvailable)
						temp = 1;

					printf("   %d.%d %-11s: %d", i + 1, j + 1, deviceAttributeNames[j], temp);
					break;
				}
				std::cout << std::endl;
			}
		}
	}

	std::cout << "Press ENTER to continue...";
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

	/* 
	
		Before creating a context, we need to select the device for which to do our DATA PARALLEL
		tasks on.

		We have two criteria for deciding which devices to use, compute units and clock speed.

		For our GPU, we just care about compute units for launching more work items.

	*/

	int max = 0;
	for (int k = 0; k < platformCount; k++)
	{
		// for each platform check each GPU device
		for (int i = 0; i < deviceCount[k]; i++) {
			// get device attribute value size
			clGetDeviceInfo(devices[k][i], CL_DEVICE_MAX_COMPUTE_UNITS, 0, NULL, &infoSize);
			// get device attribute value
			clGetDeviceInfo(devices[k][i], CL_DEVICE_MAX_COMPUTE_UNITS, infoSize, &uiNumComputeUnits, NULL);
			if (uiNumComputeUnits > max)
			{
				max = uiNumComputeUnits;
				uiTargetDevice = i;
				uiTargetPlatform = k;
			}
		}
	}
	/* Display chosen device*/
	printf("Chosen Platform: \n", uiTargetPlatform + 1);
	// get platform attribute value size
	clGetDeviceInfo(devices[uiTargetPlatform][uiTargetDevice], CL_DEVICE_NAME, 0, NULL, &infoSize);
	info = (char*)malloc(infoSize);

	// get platform attribute value
	clGetDeviceInfo(devices[uiTargetPlatform][uiTargetDevice], CL_DEVICE_NAME, infoSize, info, NULL);
	printf("   %-11s: %s\n", deviceAttributeNames[0], info);
	free(info);
	// get platform attribute value size
	clGetDeviceInfo(devices[uiTargetPlatform][uiTargetDevice], CL_DEVICE_MAX_COMPUTE_UNITS, 0, NULL, &infoSize);

	// get platform attribute value
	clGetDeviceInfo(devices[uiTargetPlatform][uiTargetDevice], CL_DEVICE_MAX_COMPUTE_UNITS, infoSize, &uiNumComputeUnits, NULL);
	printf("   %-11s: %d\n", deviceAttributeNames[0], uiNumComputeUnits);
	std::cout << "Press ENTER to continue...";
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

	/*

	Before creating a context, we need to select the device for which to do our physics
	computations. This I will choose to do on the CPU as the functions are O(N) in size
	regardless (where N is either the number of objects or the number of detected
	collisions. I believe this to be effectively data parallel, but the maths and comparison
	statements make me think the CPU may be best? It's hard to say. Requires profiling.

	We have two criteria for deciding which devices to use, compute units and clock speed.

	For our CPU, we just care about max clockspeed for doing computations quickly.

	Although most likely most computers have only 1 CPU, but we never know, so lets be safe.

	
	*/

	max = maxFrequency = infoSize = 0;
	std::cout << "Finding best CPU" << std::endl;
	for (int k = 0; k < platformCount; k++)
	{
		// for each platform check each GPU device
		for (int i = 0; i < CPUDeviceCount[k]; i++) {
			// get device attribute value size
			clGetDeviceInfo(CPUDevices[k][i], CL_DEVICE_MAX_CLOCK_FREQUENCY, 0, NULL, &infoSize);
			// get device attribute value
			clGetDeviceInfo(CPUDevices[k][i], CL_DEVICE_MAX_CLOCK_FREQUENCY, infoSize, &maxFrequency, NULL);
			if (maxFrequency > max)
			{
				max = maxFrequency;
				uiTargetCPUDevice = i;
				uiTargetCPUPlatform = k;
			}
		}
	}
	/* Display chosen device*/
	printf("Chosen Platform: \n", uiTargetCPUPlatform + 1);
	// get platform attribute value size
	try
	{
		clGetDeviceInfo(CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice], CL_DEVICE_NAME, 0, NULL, &infoSize);
		info = (char*)malloc(infoSize);
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception: IntegrateForce: " << e.what() << std::endl;
	}

	// get platform attribute value
	clGetDeviceInfo(CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice], CL_DEVICE_NAME, infoSize, info, NULL);
	printf("   %-11s: %s\n", deviceAttributeNames[0], info);
	free(info);
	// get platform attribute value size
	clGetDeviceInfo(CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice], CL_DEVICE_MAX_CLOCK_FREQUENCY, 0, NULL, &infoSize);

	// get platform attribute value
	clGetDeviceInfo(CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice], CL_DEVICE_MAX_CLOCK_FREQUENCY, infoSize, &maxFrequency, NULL);
	printf("   %-11s: %d\n", deviceAttributeNames[3], maxFrequency);
	std::cout << "Press ENTER to continue...";
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

	/* GPU */
	/* Create OpenCL context */
	context = clCreateContext(NULL, 1, &devices[uiTargetPlatform][uiTargetDevice], NULL, NULL, &ret);
	if (ret < 0)
		return false;

	/* Create Command Queue */
	command_queue = clCreateCommandQueue(context, devices[uiTargetPlatform][uiTargetDevice], 0, &ret);
	if (ret < 0)
		return false;

	/* CPU */
	/* Create OpenCL context */
	CPUcontext = clCreateContext(NULL, 1, &CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice], NULL, NULL, &ret);
	if (ret < 0)
		return false;

	/* Create Command Queue */
	CPUcommand_queue = clCreateCommandQueue(CPUcontext, CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice], 0, &ret);
	if (ret < 0)
		return false;

	int maxPairs = (sizeWidgets*sizeWidgets);
	d_widgets = (widget*)malloc(sizeWidgets * sizeof(widget));
	d_contacts = (manifold_cl*)malloc(maxPairs * sizeof(manifold_cl));

	bVec3 tempVec;
	tempVec.x = -1;
	tempVec.y = -1;
	tempVec.z = -1;

	for (int i = 0; i < maxPairs; i++)
	{
		manifold_cl _m;
		_m.index = -1;
		_m.isColliding_cl = false;
		_m.A_cl = -1; // widget_A_index;
		_m.B_cl = -1; // widget_B_index;
		_m.gid = -1;
		_m.sf_cl = -1;
		_m.df_cl = -1;
		_m.penetration_cl = -1;
		_m.e_cl = -1;
		_m.normal_cl = tempVec;      // From A to B
		d_contacts[i] = _m;
	}

	char string[MEM_SIZE];
	fileName = "broadphase.cl";
	/* Load the source code containing the kernel*/
	fopen_s(&fp, fileName, "r");
	if (!fp) {
		fprintf(stderr, "Failed to load kernel.\n");
		exit(1);
	}
	source_str = (char*)malloc(MAX_SOURCE_SIZE);
	source_size = fread(source_str, 1, MAX_SOURCE_SIZE, fp);
	fclose(fp);


	/* Create Kernel Program from the source */
	program = clCreateProgramWithSource(context, 1, (const char **)&source_str,
		(const size_t *)&source_size, &ret);
	if (ret < 0)
		return false;

	/* Create CPU Kernel Program from the source */
	CPUprogram = clCreateProgramWithSource(CPUcontext, 1, (const char **)&source_str,
		(const size_t *)&source_size, &ret);
	if (ret < 0)
		return false;

	/* Build Kernel Program */
	ret = clBuildProgram(program, 1, &devices[uiTargetPlatform][uiTargetDevice], NULL, NULL, NULL);
	// First call to know the proper size
	// build failed
	if (ret != CL_SUCCESS) {

		// check build error and build status first
		clGetProgramBuildInfo(program, devices[uiTargetPlatform][uiTargetDevice], CL_PROGRAM_BUILD_STATUS,
			sizeof(cl_build_status), &status, NULL);

		// check build log
		clGetProgramBuildInfo(program, devices[uiTargetPlatform][uiTargetDevice],
			CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize);
		programLog = (char*)calloc(logSize + 1, sizeof(char));
		clGetProgramBuildInfo(program, devices[uiTargetPlatform][uiTargetDevice],
			CL_PROGRAM_BUILD_LOG, logSize + 1, programLog, NULL);
		printf("Build failed; error=%d, status=%d, programLog:nn%s",
			ret, status, programLog);
		free(programLog);
		std::cout << "Press ENTER to continue...";
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	if (ret < 0)
		return false;

	/* Build CPU Kernel Program */
	ret = clBuildProgram(CPUprogram, 1, &CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice], NULL, NULL, NULL);
	// First call to know the proper size
	// build failed
	if (ret != CL_SUCCESS) {

		// check build error and build status first
		clGetProgramBuildInfo(CPUprogram, CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice], CL_PROGRAM_BUILD_STATUS,
			sizeof(cl_build_status), &status, NULL);

		// check build log
		clGetProgramBuildInfo(CPUprogram, CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice],
			CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize);
		programLog = (char*)calloc(logSize + 1, sizeof(char));
		clGetProgramBuildInfo(CPUprogram, CPUDevices[uiTargetCPUPlatform][uiTargetCPUDevice],
			CL_PROGRAM_BUILD_LOG, logSize + 1, programLog, NULL);
		printf("Build failed; error=%d, status=%d, programLog:nn%s",
			ret, status, programLog);
		free(programLog);
		std::cout << "Press ENTER to continue...";
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	if (ret < 0)
		return false;

	/* Create OpenCL Kernel */
	kernel = clCreateKernel(program, "broadphase_kernel", &ret);
	if (ret < 0)
		return false;

	/* Create OpenCL Kernel */
	CPUkernel_0 = clCreateKernel(CPUprogram, "compute_physics_widgets_kernel", &ret);
	if (ret < 0)
		return false;
	/* Create OpenCL Kernel */
	CPUkernel_1 = clCreateKernel(CPUprogram, "compute_physics_contacts_kernel", &ret);
	if (ret < 0)
		return false;

	/* Create Memory Buffer */
	memobj_widgets = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeWidgets * sizeof(widget), NULL, &ret);
	if (ret < 0)
		return false;

	memobj_contacts = clCreateBuffer(context, CL_MEM_READ_WRITE, maxPairs * sizeof(manifold_cl), NULL, &ret);
	if (ret < 0)
		return false;

	memobj_contacts_size = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;
	memobj_widgets_size = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;

	/* Create CPU Memory Buffer */
	cpu_memobj_widgets = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeWidgets * sizeof(widget), NULL, &ret);
	if (ret < 0)
		return false;

	cpu_memobj_contacts = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, maxPairs * sizeof(manifold_cl), NULL, &ret);
	if (ret < 0)
		return false;
	cpu_memobj_widgets_size = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;

	cpu_memobj_contacts_size = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;
	cpu_memobj_gravity = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeof(cl_float), NULL, &ret);
	if (ret < 0)
		return false;
	cpu_memobj_dt = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeof(cl_float), NULL, &ret);
	if (ret < 0)
		return false;


	return true;
}

bool bPhysics::resizeBuffers_CL(int sizeWidgets)
{
	int max = (sizeWidgets*sizeWidgets);
	//widget* temp = d_widgets;
	d_widgets = (widget*)realloc(d_widgets, sizeWidgets * sizeof(widget));

	//manifold_cl* temp2 = d_contacts;
	d_contacts = (manifold_cl*)realloc(d_contacts, max * sizeof(manifold_cl));

	bVec3 tempVec;
	tempVec.x = -1;
	tempVec.y = -1;
	tempVec.z = -1;

	for (int i = 0; i < max; i++)
	{
		manifold_cl _m;
		_m.index = -1;
		_m.isColliding_cl = false;
		_m.A_cl = -1; // widget_A_index;
		_m.B_cl = -1; // widget_B_index;
		_m.gid = -1;
		_m.sf_cl = -1;
		_m.df_cl = -1;
		_m.penetration_cl = -1;
		_m.e_cl = -1;
		_m.normal_cl = tempVec;      // From A to B
		d_contacts[i] = _m;
	}

	/* Create Memory Buffer */
	memobj_widgets = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeWidgets * sizeof(widget), NULL, &ret);
	if (ret < 0)
		return false;

	memobj_contacts = clCreateBuffer(context, CL_MEM_READ_WRITE, max * sizeof(manifold_cl), NULL, &ret);
	if (ret < 0)
		return false;

	memobj_contacts_size = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;
	memobj_widgets_size = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;

	/* Create CPU Memory Buffer */
	cpu_memobj_contacts = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, max * sizeof(manifold_cl), NULL, &ret);
	if (ret < 0)
		return false;
	cpu_memobj_widgets = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeWidgets * sizeof(widget), NULL, &ret);
	if (ret < 0)
		return false;
	cpu_memobj_contacts_size = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;
	cpu_memobj_widgets_size = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;
	cpu_memobj_gravity = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeof(cl_float), NULL, &ret);
	if (ret < 0)
		return false;
	cpu_memobj_dt = clCreateBuffer(CPUcontext, CL_MEM_READ_WRITE, sizeof(cl_float), NULL, &ret);
	if (ret < 0)
		return false;
}

bool bPhysics::updateCPUBuffers_CL(widget*& _widgets, manifold_cl*& _contacts, int sizeWidgets, int sizeContacts)
{
	if (sizeWidgets >= sizeContacts)
	{
		current = CPUkernel_0;
	}
	else
	{
		current = CPUkernel_1;
	}

	int _maxPairs = (sizeWidgets * sizeWidgets);
	/* Copy input data to the memory buffer */
	ret = clEnqueueWriteBuffer(CPUcommand_queue, cpu_memobj_widgets, CL_TRUE, 0, sizeWidgets * sizeof(widget), _widgets, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(CPUcommand_queue, cpu_memobj_contacts, CL_TRUE, 0, _maxPairs * sizeof(manifold_cl), _contacts, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(CPUcommand_queue, cpu_memobj_widgets_size, CL_TRUE, 0, sizeof(int), &sizeWidgets, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(CPUcommand_queue, cpu_memobj_contacts_size, CL_TRUE, 0, sizeof(int), &sizeContacts, 0, NULL, NULL);
	if (ret < 0)
		return false;

	ret = clEnqueueWriteBuffer(CPUcommand_queue, cpu_memobj_gravity, CL_TRUE, 0, sizeof(cl_float), &gravity.y, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(CPUcommand_queue, cpu_memobj_dt, CL_TRUE, 0, sizeof(cl_float), &dt, 0, NULL, NULL);
	if (ret < 0)
		return false;
	/* Set OpenCL Kernel Parameters */
	ret = clSetKernelArg(current, 0, sizeof(cl_mem), (void *)&cpu_memobj_widgets);
	if (ret < 0)
		return false;

	ret = clSetKernelArg(current, 1, sizeof(cl_mem), (void *)&cpu_memobj_contacts);
	if (ret < 0)
		return false;
	ret = clSetKernelArg(current, 2, sizeof(int), &sizeWidgets);
	if (ret < 0)
		return false;
	ret = clSetKernelArg(current, 3, sizeof(int), &sizeContacts);
	if (ret < 0)
		return false;
	ret = clSetKernelArg(current, 4, sizeof(cl_float), &gravity.y);
	if (ret < 0)
		return false;
	ret = clSetKernelArg(current, 5, sizeof(cl_float), &dt);
	if (ret < 0)
		return false;
}

bool bPhysics::updateBuffers_CL(widget*& _widgets, int sizeWidgets, int& sizeContacts, manifold_cl*& _contacts)
{
	int _maxPairs = (sizeWidgets * sizeWidgets);
	/* Copy input data to the memory buffer */
	ret = clEnqueueWriteBuffer(command_queue, memobj_widgets, CL_TRUE, 0, sizeWidgets * sizeof(widget), _widgets, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(command_queue, memobj_contacts, CL_TRUE, 0, _maxPairs * sizeof(manifold_cl), _contacts, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(command_queue, memobj_widgets_size, CL_TRUE, 0, sizeof(int), &sizeWidgets, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(command_queue, memobj_contacts_size, CL_TRUE, 0, sizeof(int), &sizeContacts, 0, NULL, NULL);
	if (ret < 0)
		return false;

	/* Set OpenCL Kernel Parameters */
	ret = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&memobj_widgets);
	if (ret < 0)
		return false;

	ret = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void *)&memobj_contacts);
	if (ret < 0)
		return false;
	ret = clSetKernelArg(kernel, 2, sizeof(int), &sizeWidgets);
	if (ret < 0)
		return false;
	ret = clSetKernelArg(kernel, 3, sizeof(cl_mem), (void *)&memobj_contacts_size);
	if (ret < 0)
		return false;
}

bool bPhysics::runComputePhysics_CL(int numWidgets, int numContacts, widget*& _widgets, manifold_cl*& _contacts)
{
	/* Declare our work group dimensionality */
	const size_t dimSize = 1;
	size_t global_item_size[dimSize];

	if (numWidgets >= numContacts)
	{
		global_item_size[0] = numWidgets;
		//global_item_size[1] = numWidgets;
	}
	else
	{
		global_item_size[0] = numContacts;
		//global_item_size[1] = numContacts;
	}

	/* Execute OpenCL kernel as data parallel */
	ret = clEnqueueNDRangeKernel(CPUcommand_queue, current, dimSize, NULL, global_item_size, NULL, 0, NULL, NULL);
	if (ret < 0)
		return false;

	/* Copy results from the memory buffer */
	/* Transfer result to host */
	ret = clEnqueueReadBuffer(CPUcommand_queue, cpu_memobj_widgets, CL_TRUE, 0, numWidgets * sizeof(widget), _widgets, 0, NULL, NULL);
	ret = clEnqueueReadBuffer(CPUcommand_queue, cpu_memobj_contacts, CL_TRUE, 0, numContacts * sizeof(manifold_cl), _contacts, 0, NULL, NULL);
	if (ret < 0)
		return false;

	//ret = clFinish(command_queue); // maybe sync?
	if (ret < 0)
		return false;
}

bool bPhysics::runGeneratePairsCL(widget*& _widgets, int sizeWidgets, int& sizeContacts, manifold_cl*& _contacts)
{
	int maxPairs = sizeWidgets * sizeWidgets;
	/* Declare our work group dimensionality */
	const size_t dimSize = 2;
	size_t global_item_size[dimSize];
	global_item_size[0] = sizeWidgets;
	global_item_size[1] = sizeWidgets;

	/* Execute OpenCL kernel as data parallel */
	ret = clEnqueueNDRangeKernel(command_queue, kernel, dimSize, NULL, global_item_size, NULL, 0, NULL, NULL);
	if (ret < 0)
		return false;

	/* Copy results from the memory buffer */
	/* Transfer result to host */
	ret = clEnqueueReadBuffer(command_queue, memobj_widgets, CL_TRUE, 0, sizeWidgets * sizeof(widget), _widgets, 0, NULL, NULL);
	ret = clEnqueueReadBuffer(command_queue, memobj_contacts, CL_TRUE, 0, maxPairs * sizeof(manifold_cl), _contacts, 0, NULL, NULL);
	ret = clEnqueueReadBuffer(command_queue, memobj_contacts_size, CL_TRUE, 0, sizeof(int), &sizeContacts, 0, NULL, NULL);
	if (ret < 0)
		return false;

	//ret = clFinish(command_queue); // maybe sync?
	if (ret < 0)
		return false;

	/* Sort & Display Result */
	temp_contacts = (manifold_cl*)malloc(maxPairs * sizeof(manifold_cl));
	int counter = 0;
	int inv_counter = maxPairs - 1;

	for (int i = 0; i < maxPairs; i++)
	{
		if (_contacts[i].index > -1)
		{
			temp_contacts[counter] = _contacts[i];
			counter++;
		}
		else
		{
			temp_contacts[inv_counter] = _contacts[i];
			inv_counter--;
		}

	}

	for (int i = 0; i < sizeContacts; i++)
	{
		_contacts[i] = temp_contacts[i];
	}
	free(temp_contacts);
	return true;
}

bool bPhysics::OpenCLArrayTest()
{
	cl_context context = NULL;				// OpenCL Context
	cl_command_queue command_queue = NULL;	// OpenCL Command Queue
	cl_mem memobj_a = NULL;
	cl_mem memobj_a_size = NULL;
	cl_mem memobj_b = NULL;
	cl_mem memobj_b_size = NULL;
	cl_program program = NULL;
	cl_kernel kernel = NULL;
	cl_device_id *cdDevices = NULL;     // OpenCL device list
	cl_platform_id platform_id = NULL;		// OpenCL Platform
	cl_uint ret_num_devices;
	cl_uint ret_num_platforms;
	cl_int ret;
	cl_uint uiNumComputeUnits;
	cl_uint uiTargetDevice = 0;	        // OpenCL Device to compute on

	FILE *fp;
	char *fileName;
	char *source_str;
	size_t source_size;

	int _size = 10;
	int d_b_max_size = _size * _size;

	// d_a = device_array
	int *d_a = (int*)malloc(_size * sizeof(int));
	// d_a_size = device_array, the number of elements we pass in.
	int d_a_size = _size;

	// init values for d_a
	for (int i = 0; i < _size; i++)
	{
		d_a[i] = i;
	}

	int *d_b = (int*)malloc(d_b_max_size * sizeof(int));
	int *d_b_size = (int*)malloc(sizeof(int));

	// init values for d_b
	for (int i = 0; i < _size; i++)
	{
		d_b[i] = -1;
	}
	d_b_size[0] = 0;

	/* Get Platform and Device Info */
	ret = clGetPlatformIDs(1, &platform_id, &ret_num_platforms);
	if (ret < 0)
		return false;

	ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_GPU, 0, NULL, &ret_num_devices);
	//ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_DEFAULT, 1, &device_id, &ret_num_devices);
	if (ret < 0)
		return false;

	std::cout << " # of devices = " << ret_num_devices << std::endl;
	cdDevices = (cl_device_id*)malloc(ret_num_devices * sizeof(cl_device_id));
	ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_GPU, ret_num_devices, cdDevices, NULL);
	if (ret < 0)
		return false;
	uiTargetDevice = glm::clamp((int)uiTargetDevice, (int)0, (int)(ret_num_devices - 1));

	std::cout << "Using device #: " << uiTargetDevice << std::endl;
	clGetDeviceInfo(cdDevices[uiTargetDevice], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(uiNumComputeUnits), &uiNumComputeUnits, NULL);
	std::cout << " # of Compute Units = " << uiNumComputeUnits << std::endl;


	/* Create OpenCL context */
	context = clCreateContext(NULL, 1, &cdDevices[uiTargetDevice], NULL, NULL, &ret);
	if (ret < 0)
		return false;

	/* Create Command Queue */
	command_queue = clCreateCommandQueue(context, cdDevices[uiTargetDevice], 0, &ret);
	if (ret < 0)
		return false;

	char string[MEM_SIZE];
	fileName = "broadphase.cl";
	/* Load the source code containing the kernel*/
	fopen_s(&fp, fileName, "r");
	if (!fp) {
		fprintf(stderr, "Failed to load kernel.\n");
		exit(1);
	}
	source_str = (char*)malloc(MAX_SOURCE_SIZE);
	source_size = fread(source_str, 1, MAX_SOURCE_SIZE, fp);
	fclose(fp);


	/* Create Kernel Program from the source */
	program = clCreateProgramWithSource(context, 1, (const char **)&source_str,
		(const size_t *)&source_size, &ret);

	if (ret < 0)
		return false;

	/* Build Kernel Program */
	ret = clBuildProgram(program, 1, &cdDevices[uiTargetDevice], NULL, NULL, NULL);
	// First call to know the proper size
	// build failed
	if (ret != CL_SUCCESS) {

		// check build error and build status first
		clGetProgramBuildInfo(program, cdDevices[uiTargetDevice], CL_PROGRAM_BUILD_STATUS,
			sizeof(cl_build_status), &status, NULL);

		// check build log
		clGetProgramBuildInfo(program, cdDevices[uiTargetDevice],
			CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize);
		programLog = (char*)calloc(logSize + 1, sizeof(char));
		clGetProgramBuildInfo(program, cdDevices[uiTargetDevice],
			CL_PROGRAM_BUILD_LOG, logSize + 1, programLog, NULL);
		printf("Build failed; error=%d, status=%d, programLog:nn%s",
			ret, status, programLog);
		free(programLog);
		std::cout << "Press ENTER to continue...";
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	if (ret < 0)
		return false;


	/* Create OpenCL Kernel */
	kernel = clCreateKernel(program, "test_kernel", &ret);
	if (ret < 0)
		return false;

	/* Create Memory Buffer */
	memobj_a = clCreateBuffer(context, CL_MEM_READ_WRITE, _size * sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;

	memobj_b = clCreateBuffer(context, CL_MEM_READ_WRITE, d_b_max_size * sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;

	memobj_b_size = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;
	memobj_a_size = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
	if (ret < 0)
		return false;

	/* Copy input data to the memory buffer */
	ret = clEnqueueWriteBuffer(command_queue, memobj_a, CL_TRUE, 0, _size * sizeof(int), d_a, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(command_queue, memobj_b, CL_TRUE, 0, d_b_max_size * sizeof(int), d_b, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(command_queue, memobj_a_size, CL_TRUE, 0, sizeof(int), &d_a_size, 0, NULL, NULL);
	if (ret < 0)
		return false;
	ret = clEnqueueWriteBuffer(command_queue, memobj_b_size, CL_TRUE, 0, sizeof(int), d_b_size, 0, NULL, NULL);
	if (ret < 0)
		return false;

	/* Set OpenCL Kernel Parameters */
	ret = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&memobj_a);
	if (ret < 0)
		return false;

	ret = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void *)&memobj_b);
	if (ret < 0)
		return false;
	ret = clSetKernelArg(kernel, 2, sizeof(int), &_size);
	if (ret < 0)
		return false;
	ret = clSetKernelArg(kernel, 3, sizeof(cl_mem), (void *)&memobj_b_size);
	if (ret < 0)
		return false;

	/* Execute OpenCL Kernel */
	const size_t dimSize = 2;
	size_t global_item_size[dimSize];
	global_item_size[0] = _size;
	global_item_size[1] = _size;
	//size_t local_item_size = sizeWidgets;

	/* Execute OpenCL kernel as data parallel */
	ret = clEnqueueNDRangeKernel(command_queue, kernel, dimSize, NULL, global_item_size, NULL, 0, NULL, NULL);
	//ret = clEnqueueTask(command_queue, kernel, NULL, NULL, NULL);
	if (ret < 0)
		return false;

	/* Copy results from the memory buffer */
	/* Transfer result to host */
	ret = clEnqueueReadBuffer(command_queue, memobj_a, CL_TRUE, 0, _size * sizeof(int), d_a, 0, NULL, NULL);
	ret = clEnqueueReadBuffer(command_queue, memobj_b, CL_TRUE, 0, d_b_max_size * sizeof(int), d_b, 0, NULL, NULL);
	ret = clEnqueueReadBuffer(command_queue, memobj_b_size, CL_TRUE, 0, sizeof(int), d_b_size, 0, NULL, NULL);
	if (ret < 0)
		return false;

	ret = clFinish(command_queue);
	if (ret < 0)
		return false;
	/* Display Result */

	std::cout << d_b_size[0] << std::endl;
	for (int i = 0; i < d_b_max_size; i++)
	{
		std::cout << d_b[i] << std::endl;

	}

	std::cout << "Press ENTER to continue...";
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

bool bPhysics::cleanupCL()
{
	/* Finalization */
	ret = clFlush(command_queue);
	ret = clFinish(command_queue);
	ret = clReleaseKernel(kernel);
	ret = clReleaseProgram(program);
	ret = clReleaseMemObject(memobj_widgets);
	ret = clReleaseMemObject(memobj_contacts);
	ret = clReleaseMemObject(memobj_contacts_size);
	ret = clReleaseCommandQueue(command_queue);
	ret = clReleaseContext(context);
	if (ret < 0)
		return false;

	free(source_str);
}