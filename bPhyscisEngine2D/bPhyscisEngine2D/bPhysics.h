#pragma once
#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL\cl.h>
#endif
#include <exception>
#include <chrono>
#include "bWidget.h"
#include "bRigidBody2D.h"
#include "bManifold.h"
#include "bQuadTree.h"

class bPhysics {
private:
	int grainsize = 1;
	const float gravityScale = 5.0f;
	const float m_iterations = 10;
	const float fps = 60;
	float accumulator;
	float frameStart;
	bQuadTree* tree;
public:
	const float dt = 1 / fps;
	const glm::vec3 gravity = glm::vec3(0.0f, 9.80665f * gravityScale, 0.0f);
	bVec3 bGravity;
	const float EPSILON = 0.0001f;
	std::vector<bRigidbody2D*> bodies;
	std::vector<bManifold> contacts;
	bManifold* contacts_array;
	bRigidbody2D** bodies_array;
	bPhysics();
	bPhysics(bQuadTree* _tree)
	{
		this->tree = _tree;
	};
	void FixedUpdate();
	void IntegrateForce(bRigidbody2D *b, float dt);
	void Step();
	void GeneratePairsQuadTree();
	void GeneratePairsUnthreaded();
	void IntegrateForces(bRigidbody2D *b, float dt);
	void IntegrateVelocity(bRigidbody2D *b, float dt);
	void PositionalCorrection(bManifold* m);
	void ClearForces();
	void setTree(bQuadTree* _tree) { this->tree = _tree; };
	void ComputePhysics();
	bRigidbody2D* TryGetBody(int index);

	// more threading
	//void GeneratePairs();
	//void IntegrateForcesThreaded(float dt);
	//void IntegrateVelocityThreaded(float dt);

	// OpenCL
	void GeneratePairs_CL(manifold_cl*& _contacts, widget*& _widgets, int _WidgetSize, int& _contactsSize);
	void FixedUpdate_CL(int _numWidgets, widget*& _widgets);
	void ComputePhysics_CL(int numWidgets, int& numContacts, widget*& _widgets, manifold_cl*& _contacts);
	void Step_CL(int numWidgets, widget*& _widgets);

	// more opencl
	cl_device_id device_id = NULL;
	cl_context context = NULL;				// OpenCL Context
	cl_command_queue command_queue = NULL;	// OpenCL Command Queue
	cl_mem memobj_widgets = NULL;
	cl_mem memobj_widgets_size = NULL;
	cl_mem memobj_contacts = NULL;
	cl_mem memobj_contacts_size = NULL;
	cl_program program = NULL;
	cl_kernel kernel = NULL;
	cl_device_id *cdDevices = NULL;     // OpenCL device list
	cl_uint platformCount;
	cl_platform_id *platforms;
	cl_int ret; // error code
	cl_uint uiNumComputeUnits;
	cl_uint uiTargetDevice = 0;	        // OpenCL Device to compute on
	cl_uint uiTargetPlatform = 0;
	//
	cl_uint uiTargetCPUDevice = 0;	        // OpenCL Device to compute on
	cl_uint uiTargetCPUPlatform = 0;		// CPU
	cl_context CPUcontext = NULL;				// OpenCL Context
	cl_command_queue CPUcommand_queue = NULL;	// OpenCL Command Queue
	cl_program CPUprogram = NULL;
	cl_kernel CPUkernel_0 = NULL;
	cl_kernel CPUkernel_1 = NULL;
	cl_kernel current = NULL;
	cl_mem cpu_memobj_widgets = NULL;
	cl_mem cpu_memobj_contacts = NULL;
	cl_mem cpu_memobj_widgets_size = NULL;
	cl_mem cpu_memobj_contacts_size = NULL;
	cl_mem cpu_memobj_gravity = NULL;
	cl_mem cpu_memobj_dt = NULL;
	//cl_int err;
	size_t logSize;
	char *programLog;
	cl_build_status status;

	FILE *fp;
	char *fileName;
	char *source_str;
	size_t source_size;

	widget* d_widgets;
	manifold_cl* d_contacts;
	manifold_cl* temp_contacts;
	manifold_cl* _contacts;

	bool bCL_Init(widget*& _widgets, int sizeWidgets);
	bool cleanupCL();
	bool opencl_ReadKernel(int sizeWidgets);
	bool updateBuffers_CL(widget*& _widgets, int sizeWidgets, int& sizeContacts, manifold_cl*& _contacts);
	bool runGeneratePairsCL(widget*& _widgets, int sizeWidgets, int& sizeContacts, manifold_cl*& _contacts);

	bool updateCPUBuffers_CL(widget*& _widgets, manifold_cl*& _contacts, int sizeWidgets, int sizeContacts);
	bool runComputePhysics_CL(int numWidgets, int numContacts, widget*& _widgets, manifold_cl*& _contacts);
	bool resizeBuffers_CL(int sizeWidgets);
	std::chrono::high_resolution_clock::time_point start;
	std::chrono::high_resolution_clock::time_point end;
	bool OpenCLArrayTest();
	double duration;
	char physicsTimer[256];
	void clLogging();

};