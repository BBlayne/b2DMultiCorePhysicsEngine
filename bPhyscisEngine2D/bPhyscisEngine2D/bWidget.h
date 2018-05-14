#pragma once
#include "Transform.h"
#include "bRigidBody2D.h"
#include "bShader.h"
#include "Mesh.h"

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL\cl.h>
#endif

class bWidget
{
private:
	int objectID;
public:
	Transform* myTransform;
	bRigidbody2D* myRigidBody2D;
	bShader* myShader;
	Mesh* mesh;

	void SetWidgetID(int myID);
	int getWidgetID();

	bWidget(Shape* myShape, Transform* myTransform, bRigidbody2D* myRigidBody2D);
	bWidget(Shape* myShape, Transform* myTransform, bRigidbody2D* myRigidBody2D, bShader* myShader);
	bWidget(Shape* myShape, Transform* myTransform, bRigidbody2D* myRigidBody2D, bShader* myShader, Mesh* mesh, int _id);
	~bWidget() {};

	void render();
	void InitBuffers();

};

typedef struct tag_bVec3 {
	cl_float x;
	cl_float y;
	cl_float z;
} bVec3;

typedef struct tag_transform_cl {
	bVec3 position_cl;
	cl_float orientation_in_radians_cl;
	//glm::mat4 translation_cl;
	//glm::mat4 rotation_cl;
	//glm::mat4 scale_cl;
} transform_cl;

typedef struct tag_widget {
	cl_int id_cl;

	cl_int shape_cl;
	cl_bool isStatic_cl;
	cl_float density_cl;
	cl_float inv_intertia_cl;
	cl_float inertia_cl;
	cl_float inv_mass_cl;
	cl_float mass_cl;
	bVec3 force_cl;
	bVec3 velocity_cl;
	//glm::vec3 force;
	//glm::vec3 velocity;
	cl_float angularVelocity_cl;
	cl_float torque_cl;
	cl_float impulse_cl;
	cl_float restitution_cl;
	cl_float staticFriction_cl;
	cl_float dynamicFriction_cl;
	cl_float width_cl;
	cl_float height_cl;
	transform_cl t_cl;
} widget;

// manifold
typedef struct tag_manifold_cl {
	cl_int A_cl; // widget_A_index;
	cl_int B_cl; // widget_B_index;
	cl_int gid;
	cl_int index;
	cl_float sf_cl;
	cl_float df_cl;
	cl_float penetration_cl;
	cl_float e_cl;
	bVec3 normal_cl;      // From A to B
	//glm::vec3 normal;
	cl_bool isColliding_cl;
} manifold_cl;