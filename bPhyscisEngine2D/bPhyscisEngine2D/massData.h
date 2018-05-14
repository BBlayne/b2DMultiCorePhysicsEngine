#pragma once

class massData {
public:
	float mass;
	float inv_mass;

	// For rotations (not covered in this article)
	float inertia;
	float inverse_inertia;
};