#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable
#pragma OPENCL EXTENSION cl_khr_local_int32_base_atomics : enable

#define LOCK(a) atom_cmpxchg(a, 0, 1)
#define UNLOCK(a) atom_cmpxchg(a, 0)

typedef struct tag_bVec3 {
	float x;
	float y;
	float z;
} bVec3;

typedef struct tag_transform_cl {
	bVec3 position_cl;
	float orientation_in_radians_cl;
} transform_cl;

typedef struct tag_widget {
	int id_cl;

	int shape_cl; //0 poly, 1 circle
	bool isStatic_cl;
	float density_cl;
	float inv_intertia_cl;
	float inertia_cl;
	float inv_mass_cl;
	float mass_cl;
	bVec3 force_cl;
	bVec3 velocity_cl;
	float angularVelocity_cl;
	float torque_cl;
	float impulse_cl;
	float restitution_cl;
	float staticFriction_cl;
	float dynamicFriction_cl;
	float width_cl;
	float height_cl;
	transform_cl t_cl;
} widget;

// manifold
typedef struct tag_manifold_cl {
	int A_cl;
	int B_cl;
	int gid;
	int index;

	float sf_cl;
	float df_cl;
	float penetration_cl;
	float e_cl;
	bVec3 normal_cl;      // From A to B
	bool isColliding_cl;
} manifold_cl;

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

float bAbs(float _number)
{
	if (_number < 0)
	{
		return -_number;
	}
	else
	{
		return -_number;
	}
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

float bClamp(float a, float MIN, float MAX)
{
	a = (a > MAX) ? MAX : ((a < MIN) ? MIN : a);
	return a;
}

float bExp(float _base)
{
	return _base * _base;
}


float bLength2(bVec3 _a)
{
	return bExp(_a.x) + bExp(_a.y) + bExp(_a.z);
}

__kernel void test_kernel(__global int *device_a_array, __global int *device_b_array, int _aSize, __global int *_bFinalSize)
{
	int i = get_global_id(0);
	int j = get_global_id(1);

	// map a 2D array index to a 1D array
	int index = (_aSize * j) + i;

	// pass the computer index value to device_b_array at position index
	device_b_array[index] = index;

	// increment "final size" to get count of all kernels that did work
	atom_inc(&_bFinalSize[0]);
}

__kernel void quicksort_kernel(__global widget *_widgets, __global manifold_cl *_contacts, int _WidgetSize, __global int* _contactsSize)
{
	int i = get_global_id(0);
	int j = get_global_id(1);


}

__kernel void compute_physics_contacts_kernel(__global widget *_widgets, __global manifold_cl *_contacts, int _WidgetSize, int _contactsSize, float dt, float gravity)
{
	bVec3 bGravity;
	bGravity.x = 0;
	bGravity.y = gravity;
	bGravity.z = 0;

	// the number of threads is no larger than the total number of objects on screen
	int i = get_global_id(0);
	// might need offset for local work items

	if (i < _WidgetSize)
	{
		if (_widgets[i].inv_mass_cl != 0.0f)
		{
			_widgets[i].velocity_cl = bSum(bMult(bDiff(bMult(_widgets[i].force_cl, _widgets[i].impulse_cl), bGravity), (dt / 2.0f)), _widgets[i].velocity_cl);
			_widgets[i].angularVelocity_cl += _widgets[i].torque_cl * _widgets[i].inv_intertia_cl * (dt / 2.0f);
		}
	}

	// get lowest restitution
	_contacts[i].e_cl = min(_widgets[_contacts[i].A_cl].restitution_cl, _widgets[_contacts[i].B_cl].restitution_cl);
	// Calculate static and dynamic friction
	_contacts[i].sf_cl = sqrt(_widgets[_contacts[i].A_cl].staticFriction_cl * _widgets[_contacts[i].B_cl].staticFriction_cl);
	_contacts[i].df_cl = sqrt(_widgets[_contacts[i].A_cl].dynamicFriction_cl * _widgets[_contacts[i].B_cl].dynamicFriction_cl);

	bVec3 rv;
	bVec3 temp = bDiff(_widgets[_contacts[i].B_cl].velocity_cl, _widgets[_contacts[i].A_cl].velocity_cl);
	rv.x = temp.x;
	rv.y = temp.y;
	rv.z = temp.z;

	float contactVel = bDot(rv, _contacts[i].normal_cl);
	if (contactVel <= 0)
	{
		float e = min(_widgets[_contacts[i].A_cl].restitution_cl, _widgets[_contacts[i].B_cl].restitution_cl);

		float onef = 1.0f;
		float numerator = -(onef + e) * contactVel;
		float j = numerator / (_widgets[_contacts[i].A_cl].inv_mass_cl + _widgets[_contacts[i].B_cl].inv_mass_cl);
		bVec3 impulse = bMult(_contacts[i].normal_cl, j);

		bVec3 negative_impulse;
		negative_impulse.x = -impulse.x;
		negative_impulse.y = -impulse.y;
		negative_impulse.z = -impulse.z;

		_widgets[_contacts[i].A_cl].velocity_cl = bSum(bMult(negative_impulse, _widgets[_contacts[i].A_cl].inv_mass_cl), _widgets[_contacts[i].A_cl].velocity_cl);
		bVec3 contactVector = bDiff(_widgets[_contacts[i].B_cl].t_cl.position_cl, _widgets[_contacts[i].A_cl].t_cl.position_cl);

		_widgets[_contacts[i].B_cl].velocity_cl = bSum(bMult(impulse, _widgets[_contacts[i].B_cl].inv_mass_cl), _widgets[_contacts[i].B_cl].velocity_cl);
		bVec3 contactVector2 = bDiff(_widgets[_contacts[i].A_cl].t_cl.position_cl, _widgets[_contacts[i].B_cl].t_cl.position_cl);
	}

	if (i < _WidgetSize)
	{
		if (_widgets[i].inv_mass_cl != 0.0f)
		{
			_widgets[i].t_cl.position_cl = bSum(bMult(_widgets[i].velocity_cl, (dt)), _widgets[i].t_cl.position_cl);
			_widgets[i].t_cl.orientation_in_radians_cl += _widgets[i].angularVelocity_cl * dt;
		}

		if (_widgets[i].inv_mass_cl != 0.0f)
		{
			_widgets[i].velocity_cl = bSum(bMult(bDiff(bMult(_widgets[i].force_cl, _widgets[i].impulse_cl), bGravity), (dt / 2.0f)), _widgets[i].velocity_cl);
			_widgets[i].angularVelocity_cl += _widgets[i].torque_cl * _widgets[i].inv_intertia_cl * (dt / 2.0f);
		}
	}

	float k_slop = 0.05f; // Penetration allowance
	float percent = 0.4f; // Penetration percentage to correct

	float tempMax = max(_contacts[i].penetration_cl - k_slop, 0.0f);
	float tempVal = (_widgets[_contacts[i].A_cl].inv_mass_cl + _widgets[_contacts[i].B_cl].inv_mass_cl);
	tempVal = tempMax / tempVal;

	bVec3 temp3 = bMult(_contacts[i].normal_cl, tempVal);
	bVec3 correction = bMult(temp3, percent);

	_widgets[_contacts[i].A_cl].t_cl.position_cl = bDiff(_widgets[_contacts[i].A_cl].t_cl.position_cl, bMult(correction, _widgets[_contacts[i].A_cl].inv_mass_cl));
	_widgets[_contacts[i].B_cl].t_cl.position_cl = bSum(bMult(correction, _widgets[_contacts[i].B_cl].inv_mass_cl), _widgets[_contacts[i].B_cl].t_cl.position_cl);
	
	if (i < _WidgetSize)
	{
		bVec3 temp_clear_all_forces;
		temp_clear_all_forces.x = 0;
		temp_clear_all_forces.y = 0;
		temp_clear_all_forces.z = 0;
		_widgets[i].force_cl = temp_clear_all_forces;
	}
}

__kernel void compute_physics_widgets_kernel(__global widget *_widgets, __global manifold_cl *_contacts, int _WidgetSize, int _contactsSize, float gravity, float dt)
{
	bVec3 bGravity;
	bGravity.x = 0;
	bGravity.y = gravity;
	bGravity.z = 0;
	// the number of threads is no larger than the total number of objects on screen
	int i = get_global_id(0);
	// might need offset for local work items
	if (_widgets[i].inv_mass_cl != 0.0f)
	{
		_widgets[i].velocity_cl = bSum(bMult(bDiff(bMult(_widgets[i].force_cl, _widgets[i].impulse_cl), bGravity), (dt / 2.0f)), _widgets[i].velocity_cl);
		_widgets[i].angularVelocity_cl += _widgets[i].torque_cl * _widgets[i].inv_intertia_cl * (dt / 2.0f);
	}

	if (i < _contactsSize)
	{
		// get lowest restitution
		_contacts[i].e_cl = min(_widgets[_contacts[i].A_cl].restitution_cl, _widgets[_contacts[i].B_cl].restitution_cl);
		// Calculate static and dynamic friction
		_contacts[i].sf_cl = sqrt(_widgets[_contacts[i].A_cl].staticFriction_cl * _widgets[_contacts[i].B_cl].staticFriction_cl);
		_contacts[i].df_cl = sqrt(_widgets[_contacts[i].A_cl].dynamicFriction_cl * _widgets[_contacts[i].B_cl].dynamicFriction_cl);
		
		bVec3 rv;
		bVec3 temp = bDiff(_widgets[_contacts[i].B_cl].velocity_cl, _widgets[_contacts[i].A_cl].velocity_cl);
		rv.x = temp.x;
		rv.y = temp.y;
		rv.z = temp.z;

		float contactVel = bDot(rv, _contacts[i].normal_cl);
		if (contactVel <= 0)
		{
			float e = min(_widgets[_contacts[i].A_cl].restitution_cl, _widgets[_contacts[i].B_cl].restitution_cl);

			float numerator = -(1.0f + e) * contactVel;
			float j = numerator / (_widgets[_contacts[i].A_cl].inv_mass_cl + _widgets[_contacts[i].B_cl].inv_mass_cl);
			bVec3 impulse = bMult(_contacts[i].normal_cl, j);//  *1.0f;
													 
			bVec3 negative_impulse;
			negative_impulse.x = -impulse.x;
			negative_impulse.y = -impulse.y;
			negative_impulse.z = -impulse.z;

			_widgets[_contacts[i].A_cl].velocity_cl = bSum(bMult(negative_impulse, _widgets[_contacts[i].A_cl].inv_mass_cl), _widgets[_contacts[i].A_cl].velocity_cl);
			bVec3 contactVector = bDiff(_widgets[_contacts[i].B_cl].t_cl.position_cl, _widgets[_contacts[i].A_cl].t_cl.position_cl);

			_widgets[_contacts[i].B_cl].velocity_cl = bSum(bMult(impulse, _widgets[_contacts[i].B_cl].inv_mass_cl), _widgets[_contacts[i].B_cl].velocity_cl);
			bVec3 contactVector2 = bDiff(_widgets[_contacts[i].A_cl].t_cl.position_cl, _widgets[_contacts[i].B_cl].t_cl.position_cl);
		}
	}

	if (_widgets[i].inv_mass_cl != 0.0f)
	{
		_widgets[i].t_cl.position_cl = bSum(bMult(_widgets[i].velocity_cl, (dt)), _widgets[i].t_cl.position_cl);
		_widgets[i].t_cl.orientation_in_radians_cl += _widgets[i].angularVelocity_cl * dt;
	}

	if (_widgets[i].inv_mass_cl != 0.0f)
	{
		_widgets[i].velocity_cl = bSum(bMult(bDiff(bMult(_widgets[i].force_cl, _widgets[i].impulse_cl), bGravity), (dt / 2.0f)), _widgets[i].velocity_cl);
		_widgets[i].angularVelocity_cl += _widgets[i].torque_cl * _widgets[i].inv_intertia_cl * (dt / 2.0f);
	}

	if (i < _contactsSize)
	{
		float k_slop = 0.05f; // Penetration allowance
		float percent = 0.4f; // Penetration percentage to correct

		float tempMax = max(_contacts[i].penetration_cl - k_slop, 0.0f);
		float tempVal = (_widgets[_contacts[i].A_cl].inv_mass_cl + _widgets[_contacts[i].B_cl].inv_mass_cl);
		tempVal = tempMax / tempVal;

		bVec3 temp3 = bMult(_contacts[i].normal_cl, tempVal);
		bVec3 correction = bMult(temp3, percent);

		_widgets[_contacts[i].A_cl].t_cl.position_cl = bDiff(_widgets[_contacts[i].A_cl].t_cl.position_cl, bMult(correction, _widgets[_contacts[i].A_cl].inv_mass_cl));
		_widgets[_contacts[i].B_cl].t_cl.position_cl = bSum(bMult(correction, _widgets[_contacts[i].B_cl].inv_mass_cl), _widgets[_contacts[i].B_cl].t_cl.position_cl);
	}

	bVec3 temp_clear_all_forces;
	temp_clear_all_forces.x = 0;
	temp_clear_all_forces.y = 0;
	temp_clear_all_forces.z = 0;
	_widgets[i].force_cl = temp_clear_all_forces;
}

__kernel void broadphase_kernel(__global widget *_widgets, __global manifold_cl *_contacts, int _WidgetSize, __global int* _contactsSize)
{

	int i = get_global_id(0);
	int j = get_global_id(1);
	int index;
	index = (_WidgetSize * j) + i;

	manifold_cl _m;
	bVec3 temp;
	temp.x = 0;
	temp.y = 0;
	temp.z = 0;
	_m.normal_cl = temp;
	_m.penetration_cl = 0;
	_m.A_cl = i;
	_m.B_cl = j;
	_m.isColliding_cl = false;
	_m.sf_cl = 0;
	_m.df_cl = 0;
	_m.e_cl = 0;
	_m.index = -1;
	_m.gid = index;

	widget temp_widget_a_cl;
	temp_widget_a_cl = _widgets[i];

	widget temp_widget_b_cl;
	temp_widget_b_cl = _widgets[j];

	_contacts[index] = _m;

	if ((temp_widget_a_cl.shape_cl > 0) || (temp_widget_b_cl.shape_cl > 0))
	{
		if (temp_widget_a_cl.id_cl != temp_widget_b_cl.id_cl)
		{
			bVec3 temp;
			temp.x = 0;
			temp.y = 0;
			temp.z = 0;
			_m.normal_cl = temp;
			_m.penetration_cl = 0;
			_m.A_cl = i;
			_m.B_cl = j;

			if (temp_widget_a_cl.shape_cl != 0)
			{
				if (temp_widget_b_cl.shape_cl == 1)
				{
					// CircleCircle
					bVec3 _normal = bDiff(temp_widget_b_cl.t_cl.position_cl, temp_widget_a_cl.t_cl.position_cl);

					float r = (temp_widget_a_cl.width_cl) + (temp_widget_b_cl.width_cl);
					float temp = bLength2(_normal);
					if (temp > bExp(r))
					{
						// does not contact
						_m.isColliding_cl = false;
					}
					else
					{
						// Circles have collided, compute manifold.
						float distance = sqrt(bLength2(_normal));

						// If distance between circles is not zero
						if (distance != 0)
						{
							// Penetration Distance is difference between radius and distance
							_m.penetration_cl = r - distance;
							// Utilize our distance since we performed sqrt on it already within Length( )
							// Points from A to B, and is a unit vector
							_m.normal_cl = bDiv(_normal, distance);
							_m.isColliding_cl = true;
						}
						else // circles are on the same position (What does this mean?)
						{
							_m.penetration_cl = temp_widget_a_cl.width_cl / 2;
							bVec3 _temp;
							_temp.x = 1.0f;
							_temp.y = 0;
							_temp.z = 0;
							_m.normal_cl = _temp;
							_m.isColliding_cl = true;
						}
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

					bVec3 min = bSum(temp_widget_b_cl.t_cl.position_cl, _temp_min);

					bVec3 _temp_max;
					_temp_max.x = (temp_widget_b_cl.width_cl / 2);
					_temp_max.y = (temp_widget_b_cl.height_cl / 2);
					_temp_max.z = 0;
					bVec3 max = bSum(temp_widget_b_cl.t_cl.position_cl, _temp_max);

					// Calculate half extents along each axis
					float x_extent = (max.x - min.x) / 2;
					float y_extent = (max.y - min.y) / 2;

					// Clamp point to edges of the AABB
					closest.x = bClamp(closest.x, -x_extent, x_extent);
					closest.y = bClamp(closest.y, -y_extent, y_extent);

					bool inside = false;

					// Circle is inside the AABB, so we need to clamp the circle's center
					// to the closest edge
					if (isEqual(_normal, closest))
					{
						inside = true;

						// Find closest axis
						if (bAbs(_normal.x) > bAbs(_normal.y))
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
							{
								closest.y = y_extent;
							}								
							else
							{
								closest.y = -y_extent;
							}								
						}
					}

					bVec3 normalizedNormal = bDiff(_normal, closest);
					float distance = bLength2(normalizedNormal);

					float radius = temp_widget_a_cl.width_cl;

					// Early out of the radius is shorter than distance to closest point and
					// Circle not inside the AABB
					if (!((distance > bExp(radius)) && !inside))
					{
						// Avoided sqrt until we needed
						distance = sqrt(distance);
						bVec3 negativeNormalizedNormal;
						negativeNormalizedNormal.x = -normalizedNormal.x;
						negativeNormalizedNormal.y = -normalizedNormal.y;
						negativeNormalizedNormal.z = -normalizedNormal.z;
						// Collision normal needs to be flipped to point outside if circle was
						// inside the AABB
						if (inside)
						{
							_m.normal_cl = bDiv(negativeNormalizedNormal, distance);
							_m.penetration_cl = radius - distance;
						}
						else
						{
							_m.normal_cl = bDiv(normalizedNormal, distance);
							_m.penetration_cl = radius - distance;
						}
						_m.isColliding_cl = true;
					}
				}
			}

			if (_m.isColliding_cl)
			{
				_m.index = index;
				atom_inc(&_contactsSize[0]); 
				_contacts[index] = _m;
			}
		}
	}
}