#include "Broadphase.h"

using namespace std;

void Broadphase::GeneratePairs(std::vector<bManifold> &contacts, std::vector<bBody> bodies, int threadID)
{
	std::string name = "";
	contacts.clear();

	// What I NEED to do is break this step function into two steps (haha)
	// The Manifold/Broadphase colision detection needs to be parallelized first.
	// THEN all threads can work on updating the rest of physics
	for (int i = threadID; i < bodies.size(); i++)
	{

		bBody* A = &bodies.at(i);
		if (false)
		{
			name = "Circle";

			std::cout << name << ": " << A->objID << ", Velocity: " << A->velocity.y << ", Position: "
				<< A->transform.position.x << ", " << A->transform.position.y << ", "
				<< A->transform.position.z << std::endl;
		}

		for (int j = i + 1; j < bodies.size(); ++j)
		{
			bBody* B = &bodies.at(j);
			if (A->inv_mass == 0 && B->inv_mass == 0)
				continue;

			bManifold m(A, B);
			m.Solve();
			if (m.contact_count)
			{
				//std::cout << "Contact! " << m.contact_count << std::endl;
				contacts.emplace_back(m);
			}

		}
	}
}