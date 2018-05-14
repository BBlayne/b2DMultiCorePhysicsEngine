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
			_m.A_cl = &_widgets[i];
			_m.B_cl = &_widgets[j];

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

					bVec3 min = bSum(temp_widget_b_cl.t_cl.position_cl, _temp_min);

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