#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL\cl.h>
#endif

#include <exception>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <sstream>
#include <array>
#include <vector>
#include <chrono>
#include "Dependencies\GL\glew.h"
#include "Dependencies\GL\freeglut.h"
#include "Dependencies\glfw3.h"
#include "Dependencies\Text\text2D.hpp"

// Include GLM
#include "Dependencies\glm\glm\glm.hpp"
#include "Dependencies\glm\glm\gtc\matrix_transform.hpp"
#include "Dependencies\glm\glm\gtx\transform.hpp"
#include "Dependencies\glm\glm\gtc\quaternion.hpp"
#include "Dependencies\glm\glm\gtx\quaternion.hpp"

using namespace glm;

#include "Dependencies\Shaders\shader.hpp"
#include "Dependencies\Texture\texture.hpp"

//
#include "bWidget.h"
// Body
#include "scene.h"
#include "Shape.h"
#include "bCircle.h"
#include "bRectangle.h"
#include "Transform.h"
#include "Mesh.h"
#include "bPhysics.h"
#include "bQuadTree.h"

GLFWwindow* window;

char text[256];
char text2[256];
char physicsTimer[256];
char numBodies[256];

std::vector<float> vecCirclePts;
std::vector<float> vecSqrPts;
std::vector<float> vecCrcPts;
std::vector<glm::vec2> vecCrcUVs;

bUtils _tool;

static const GLfloat poly_vertex_buffer_data[] =
{
	-20.0f, -20.0f, 0.0f,
	-20.0f, 20.0f, 0.0f,
	20.0f, 20.0f, 0.0f,
	-20.0f, -20.0f, 0.0f,
	20.0f, 20.0f, 0.0f,
	20.0f, -20.0f, 0.0f
};

static const GLfloat g_uv_buffer_data[] = {
	1.000000, 0.000000,
	1.000000, 1.000000,
	0.000000, 1.000000,
	0.000000, 1.000000,
	0.000000, 0.000000,
	1.000000, 0.000000
};

void CircleUVPts()
{
	vecCrcPts.push_back(1.000000);
	vecCrcPts.push_back(0.000000);
	vecCrcPts.push_back(1.000000);
	vecCrcPts.push_back(1.000000);
	vecCrcPts.push_back(0.000000);
	vecCrcPts.push_back(1.000000);
	vecCrcPts.push_back(0.000000);
	vecCrcPts.push_back(1.000000);
	vecCrcPts.push_back(1.000000);
	vecCrcPts.push_back(0.000000);
	vecCrcPts.push_back(0.000000);
	vecCrcPts.push_back(0.000000);
}

void CircleUVs()
{
	vecCrcUVs.push_back(glm::vec2(1.000000, 0.000000));
	vecCrcUVs.push_back(glm::vec2(1.000000, 1.000000));
	vecCrcUVs.push_back(glm::vec2(0.000000, 1.000000));
	vecCrcUVs.push_back(glm::vec2(0.000000, 0.000000));
}

void Square()
{
	vecSqrPts.push_back(-1);
	vecSqrPts.push_back(-1);
	vecSqrPts.push_back(0);
	vecSqrPts.push_back(-1);
	vecSqrPts.push_back(1);
	vecSqrPts.push_back(0);
	vecSqrPts.push_back(1);
	vecSqrPts.push_back(1);
	vecSqrPts.push_back(0);
	vecSqrPts.push_back(-1);
	vecSqrPts.push_back(-1);
	vecSqrPts.push_back(0);
	vecSqrPts.push_back(1);
	vecSqrPts.push_back(1);
	vecSqrPts.push_back(0);
	vecSqrPts.push_back(1);
	vecSqrPts.push_back(-1);
	vecSqrPts.push_back(0);
}

void WidgetToString(widget* _widget, int index)
{
	if (index >= 0 && _widget != nullptr)
		std::cout << "Position of Widget: {" << _widget[index].t_cl.position_cl.x << ", " << _widget[index].t_cl.position_cl.y << "}" << std::endl;
}

void AddRect2(glm::vec3 position, std::vector<bWidget*> &mWidgets, std::vector<bRigidbody2D*> &mBodies, int objID, float scale_width, float scale_height,
	float screen_width, float screen_height, GLuint programID, widget*& _widgets)
{
	widget temp_widget;

	bUtils mTool;
	bRigidbody2D* mRigidbody2D = new bRigidbody2D(10.0f, true);
	Transform* transform = new Transform(0, position);
	transform->translation = glm::translate(transform->position);
	transform->setScale(glm::vec3(scale_width, scale_height, 1.0f));
	bPoly* poly = new bPoly(mRigidbody2D, position, scale_width, scale_height, 1.0f, mTool.ArrayToVertices(vecSqrPts));
	//Mesh* mesh = new Mesh(g_square_buffer_data, g_square_uv_buffer_data); // test this
	Mesh* mesh = new Mesh(vecCrcUVs, vecCrcPts, vecSqrPts, vecSqrPts);
	bShader* mShader = new bShader(screen_width, screen_height);
	mShader->setProgramID(programID);
	mShader->LoadTextureDDS("SqrBrown.DDS");
	// MVP updated in scene?
	bWidget* mWidget = new bWidget(poly, transform, mRigidbody2D, mShader, mesh, objID);
	mWidget->myRigidBody2D->setParent(mWidget);
	// add to array of widgets
	mWidget->InitBuffers();
	mBodies.push_back(mRigidbody2D);
	mWidgets.push_back(mWidget);

	// struct nonsense
	temp_widget.id_cl = objID;
	bVec3 _position;
	_position.x = position.x;
	_position.y = position.y;
	_position.z = position.z;


	temp_widget.t_cl.position_cl = _position;
	//temp_widget.t_cl.translation_cl = glm::mat4(1.0f);
	//temp_widget.t_cl.rotation_cl = glm::mat4(1.0f);
	//temp_widget.t_cl.scale_cl = glm::scale(glm::vec3(UNIT_SCALE, UNIT_SCALE, 1.0f));
	temp_widget.t_cl.orientation_in_radians_cl = 0.0f;

	temp_widget.width_cl = scale_width * UNIT_SCALE * 2.5f;
	temp_widget.height_cl = scale_height * UNIT_SCALE * 2.5f;
	//temp_widget.t_cl.scale_cl = temp_widget.t_cl.scale_cl * glm::scale(glm::vec3(scale_width, scale_height, 1.0f));
	temp_widget.restitution_cl = 10.0f;
	bVec3 _force;
	_force.x = 0;
	_force.y = 0;
	_force.z = 0;

	temp_widget.force_cl = _force;
	//temp_widget.force = glm::vec3(0,0,0);
	temp_widget.torque_cl = 0.0f;
	temp_widget.density_cl = 1.0f;
	temp_widget.angularVelocity_cl = 0.0f;
	temp_widget.dynamicFriction_cl = 0.0f;
	temp_widget.impulse_cl = 0.0f;
	temp_widget.inertia_cl = 0.0f;
	temp_widget.inv_intertia_cl = 0.0f;
	temp_widget.inv_mass_cl = 0.0f;
	temp_widget.mass_cl = 0.0f;
	temp_widget.velocity_cl = _force;
	//temp_widget.velocity = glm::vec3(0, 0, 0);
	temp_widget.staticFriction_cl = 0.0f;
	temp_widget.shape_cl = 0;
	temp_widget.isStatic_cl = true;

	_widgets = (widget*)realloc(_widgets, (objID + 1) * sizeof(widget));

	_widgets[objID] = temp_widget;
}

void AddCircle2(glm::vec3 position, std::vector<bWidget*> &mWidgets, std::vector<bRigidbody2D*> &mBodies, int objID, float scale_width, float scale_height,
	float screen_width, float screen_height, GLuint programID, widget*& _widgets)
{
	widget temp_widget;
	bUtils mTool;
	bRigidbody2D* mRigidbody2D = new bRigidbody2D(1.0f, false);
	Transform* transform = new Transform(0, position);
	transform->translation = glm::translate(transform->position);
	transform->setScale(glm::vec3(scale_width, scale_height, 1.0f));
	bCircle* circle = new bCircle(mRigidbody2D, position, CIRCLE_RADIUS, 1.0f);
	Mesh* mesh = new Mesh(vecCrcUVs, vecCrcPts, vecSqrPts, vecSqrPts);
	bShader* mShader = new bShader(screen_width, screen_height);
	mShader->setProgramID(programID);
	mShader->LoadTextureDDS("CircleRed.DDS");
	// Get a handle for our "myTextureSampler" uniform
	mShader->setTexID(glGetUniformLocation(mShader->getProgramID(), "myTextureSampler"));
	// MVP updated in scene?
	bWidget* mWidget = new bWidget(circle, transform, mRigidbody2D, mShader, mesh, objID);
	mWidget->myRigidBody2D->setParent(mWidget);
	// add to array of widgets
	mWidget->InitBuffers();
	mBodies.push_back(mRigidbody2D);
	mWidgets.push_back(mWidget);

	// struct nonsense
	bVec3 _position;
	_position.x = position.x;
	_position.y = position.y;
	_position.z = position.z;


	temp_widget.id_cl = objID;
	temp_widget.t_cl.position_cl = _position;
	//temp_widget.t_cl.translation_cl = glm::mat4(1.0f);
	//temp_widget.t_cl.rotation_cl = glm::mat4(1.0f);
	//temp_widget.t_cl.scale_cl = glm::scale(glm::vec3(UNIT_SCALE, UNIT_SCALE, 1.0f));
	temp_widget.t_cl.orientation_in_radians_cl = 0.0f;
	temp_widget.width_cl = scale_width * CIRCLE_RADIUS * 2;
	temp_widget.height_cl = scale_height * CIRCLE_RADIUS * 2;
	//temp_widget.t_cl.scale_cl = temp_widget.t_cl.scale_cl * glm::scale(glm::vec3(scale_width, scale_height, 1.0f));
	temp_widget.restitution_cl = 1.0f;

	bVec3 _zero;
	_zero.x = 0;
	_zero.y = 0;
	_zero.z = 0;

	temp_widget.force_cl = _zero;
	//temp_widget.force = glm::vec3(0,0,0);
	temp_widget.torque_cl = 0.0f;
	temp_widget.density_cl = 1.0f;
	
	temp_widget.angularVelocity_cl = 0.0f;
	temp_widget.dynamicFriction_cl = 0.0f;
	temp_widget.impulse_cl = 0.0f;
	temp_widget.inertia_cl = 0.0f;
	temp_widget.inv_intertia_cl = 0.0f;
	temp_widget.inv_mass_cl = 0.0f;
	temp_widget.mass_cl = 0.0f;
	temp_widget.velocity_cl = _zero;
	//temp_widget.velocity = glm::vec3(0,0,0);
	temp_widget.staticFriction_cl = 0.0f;
	temp_widget.shape_cl = 1;
	temp_widget.isStatic_cl = false;

	temp_widget.mass_cl = 3.14159f * std::pow(CIRCLE_RADIUS, 2) * temp_widget.density_cl;
	temp_widget.inv_mass_cl = (temp_widget.mass_cl) ? 1.0f / temp_widget.mass_cl : 0.0f;
	temp_widget.inertia_cl = temp_widget.mass_cl * std::pow(CIRCLE_RADIUS, 2);
	temp_widget.inv_intertia_cl = (temp_widget.inertia_cl) ? 1.0f / temp_widget.inertia_cl : 0.0f;
	widget* temp = _widgets;
	_widgets = (widget*)realloc(temp, (objID + 1) * sizeof(widget));

	_widgets[objID] = temp_widget;
}

const float originalAspect = (float)WIDTH / (float)HEIGHT;

bool IS_PRESSED = false;
bool IS_RIGHT_PRESSED = false;

glm::vec4 ComputeAspectRatio(int w, int h, float o_ar)
{
	float n_ar = (float)w / (float)h;
	float scaleWidth = (float)w / (float)WIDTH;
	float scaleHeight = (float)h / (float)HEIGHT;
	if (n_ar > o_ar)
	{
		scaleWidth = scaleHeight;
	}
	else
	{
		scaleHeight = scaleWidth;
	}
	float marginX = (w - WIDTH * scaleWidth) / 2;
	float marginY = (h - HEIGHT * scaleHeight) / 2;

	//glm::vec4 test;
	//test.wxyz
	glm::vec4 scalePair = glm::vec4(scaleWidth, scaleHeight, marginX, marginY);

	return scalePair;
}

void OnLeftClick(int x, int y, std::vector<bWidget*> &mWidgets, std::vector<bRigidbody2D*> &mBodies, int screen_width, int screen_height, int &objID, GLuint programID, float n_ar, widget*& _widgets)
{

	glm::vec4 updatedScalePair = ComputeAspectRatio(screen_width, screen_height, originalAspect);
	
	float remapped_x = _tool.remap(x, 0, screen_width, -updatedScalePair.z, screen_width - updatedScalePair.z);
	float remapped_y = _tool.remap(y, 0, screen_height, -updatedScalePair.w, screen_height - updatedScalePair.w);

	float w_ratio = ((float)(screen_width - (updatedScalePair.z * 2)) / WIDTH);
	float h_ratio = ((float)(screen_height - updatedScalePair.w) / HEIGHT); // I guess because the height hasn't margins!?

	int newX = (remapped_x / originalAspect / w_ratio);
	int newY = (remapped_y / originalAspect / h_ratio);

	glm::vec3 pos = _tool.ScreenToWorldSpaceCoords(newX, newY, WIDTH / originalAspect, HEIGHT / originalAspect);
	AddCircle2(pos, mWidgets, mBodies, objID, 0.5f, 0.5f, screen_width, screen_height, programID, _widgets);
	sprintf_s(text, "Clicked: x: %d, y: %d", newX, newY);
}

void OnRightClick(int x, int y, std::vector<bWidget*> &mWidgets, std::vector<bRigidbody2D*> &mBodies, int screen_width, int screen_height, int &objID, GLuint programID, float n_ar, widget*& _widgets)
{

	glm::vec4 updatedScalePair = ComputeAspectRatio(screen_width, screen_height, originalAspect);

	float remapped_x = _tool.remap(x, 0, screen_width, -updatedScalePair.z, screen_width - updatedScalePair.z);
	float remapped_y = _tool.remap(y, 0, screen_height, -updatedScalePair.w, screen_height - updatedScalePair.w);

	float w_ratio = ((float)(screen_width - (updatedScalePair.z * 2)) / WIDTH);
	float h_ratio = ((float)(screen_height - updatedScalePair.w) / HEIGHT); // I guess because the height hasn't margins!?

	int newX = (remapped_x / originalAspect / w_ratio);
	int newY = (remapped_y / originalAspect / h_ratio);

	glm::vec3 pos = _tool.ScreenToWorldSpaceCoords(newX, newY, WIDTH / originalAspect, HEIGHT / originalAspect);
	AddRect2(pos, mWidgets, mBodies, objID, 1.0f, 1.0f, screen_width, screen_height, programID, _widgets);
	sprintf_s(text, "Clicked: x: %d, y: %d", newX, newY);
}

void SpawnCirclesGoldenRatioPattern(int currentObjectID, int max, std::vector<bRigidbody2D*> _bodies, std::vector<bWidget*> _widgets, int _w, int _h, GLuint _progID)
{
	int temp = 0;
	for (int i = 0; i < max; i++)
	{

		if (max == 1)
			temp = 2;
		else
			temp = max;

		float theta = 2.0f * 3.1415926f * i / (temp - 1); //the current angle
		// golden ratio to evenly space objects
		float x = i * 22 * std::cos(theta);
		float y = i * 22 * std::sin(theta);
		glm::vec3 position = glm::vec3(x, y + 65, 0);
		//AddCircle2(position, _widgets, _bodies, ++currentObjectID, 1.0f, 1.0f, _w, _h, _progID, _widgets_CL);
	}
}

void TreeFunc(bQuadTree _tree, std::vector<bWidget*> mWidgets)
{
	if (_tree.clearTree())
	{
		for (int i = 0; i < mWidgets.size(); i++)
		{
			if (i == 4)
			{
				int here = 0;
			}
			if (_tree.insert(mWidgets.at(i)))
			{
				//std::cout << "inserted " << mWidgets.at(i)->getWidgetID() << " into the tree." << std::endl;
			}
			else
			{
				//std::cout << "Error inserting into tree" << std::endl;
			}
		}
	}
	else
	{
		//std::cout << "Error clearing tree" << std::endl;
	}
}

int main()
{
	bUtils contextUtility;

	int screen_width = WIDTH;
	int screen_height = HEIGHT;
	float originalAspect = (float)screen_width / (float)screen_height;
	// Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	//glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow(screen_width, screen_height, "Tutorial", NULL, NULL);
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

	// Here we do stuff.

	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders("vertexShader.vert", "fragmentShader.frag");

	//bQuadTree* tree = new bQuadTree(0, 0, 0, WIDTH, HEIGHT);

	std::vector<bWidget*> mWidgets;
	std::vector<bRigidbody2D*> mBodies;
	widget* widgets;
	widgets = (widget*)malloc(0 * sizeof(widget));
	
	
	int zoom = 1;

	int new_height = screen_height;
	int new_width = screen_width;
	int max = 0;
	Square();
	CircleUVs();
	CircleUVPts();
	int objID = 0, temp = 0;

	float aspect = originalAspect;

	AddRect2(glm::vec3(0, (((float)HEIGHT / 2) / originalAspect), 0), mWidgets, mBodies, objID, 20.0f, 1.0f, new_width, new_height, programID, widgets);
	AddRect2(glm::vec3(-(((float)WIDTH / 2) / originalAspect), 0, 0), mWidgets, mBodies, ++objID, 1.0f, 20.0f, new_width, new_height, programID, widgets);
	AddRect2(glm::vec3(((float)WIDTH / 2) / originalAspect, 0, 0), mWidgets, mBodies, ++objID, 1.0f, 20.0f, new_width, new_height, programID, widgets);
	AddRect2(glm::vec3(0, -((float)HEIGHT / 2) / originalAspect, 0), mWidgets, mBodies, ++objID, 20.0f, 1.0f, new_width, new_height, programID, widgets);
	AddCircle2(glm::vec3(0, 0, 0), mWidgets, mBodies, ++objID, 0.5f, 0.5f, new_width, new_height, programID, widgets);
	AddCircle2(glm::vec3(50, 0, 0), mWidgets, mBodies, ++objID, 0.5f, 0.5f, new_width, new_height, programID, widgets);
	AddCircle2(glm::vec3(-50, 0, 0), mWidgets, mBodies, ++objID, 0.5f, 0.5f, new_width, new_height, programID, widgets);
	AddCircle2(glm::vec3(0, -50, 0), mWidgets, mBodies, ++objID, 0.5f, 0.5f, new_width, new_height, programID, widgets);
	AddCircle2(glm::vec3(0, 60, 0), mWidgets, mBodies, ++objID, 0.5f, 0.5f, new_width, new_height, programID, widgets);
	//AddCircle2(glm::vec3(0, 140, 0), mWidgets, mBodies, ++objID, 0.5f, 0.5f, new_width, new_height, programID, widgets);

	for (int i = 0; i < objID + 1; i++)
		WidgetToString(widgets, i);

	struct testStruct_CL {
		widget* _testWidget;
	} hey;

	hey._testWidget = &widgets[0];
	hey._testWidget->t_cl.position_cl.x += 500;
	WidgetToString(widgets, 0);
	hey._testWidget->t_cl.position_cl.x -= 500;
	WidgetToString(widgets, 0);

	scene main_scene;
	main_scene.setupScene(mWidgets);
	main_scene.setBodiesList(mBodies);
	main_scene.aspect = aspect;

	bPhysics mPhysicsController;
	//mPhysicsController.OpenCLArrayTest();
	std::cout << "Initialize OpenCL..." << std::endl;
	if(!mPhysicsController.bCL_Init(widgets, mWidgets.size()))
	{
		std::cout << "ERROR, something went wrong with initializing OpenCL" << std::endl;
		std::cout << "Press ENTER to continue...";
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	mPhysicsController.bodies = mBodies;

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	// time measurement
	contextUtility.setLastTime(glfwGetTime());

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//std::cout << "Press ENTER to continue...";
	//std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	std::vector<bWidget*> tempWidgets;
	std::vector<bRigidbody2D*> tempBodies;

	// Load the texture
	GLuint Texture = loadDDS("uvmap.DDS");

	// Get a handle for our "myTextureSampler" uniform
	GLuint TextureID = glGetUniformLocation(programID, "myTextureSampler");
	// Initialize our little text library with the Holstein font
	initText2D("Holstein.DDS");
	sprintf_s(text, "_");
	sprintf_s(text2, "_");
	//tree->debugRenderInit();

	std::chrono::high_resolution_clock::time_point start;
	std::chrono::high_resolution_clock::time_point end;

	do 
	{
		double x, y;
		glfwGetCursorPos(window, &x, &y);

		contextUtility.measureFPS();
		glfwGetWindowSize(window, &new_width, &new_height);		
		float newAspect = (float)new_width / (float)new_height;
		main_scene.aspect = 1;

		glm::vec4 updatedScalePair = ComputeAspectRatio(new_width, new_height, originalAspect);
		//wxyz
		glViewport(updatedScalePair.z, updatedScalePair.w, WIDTH * updatedScalePair.x, HEIGHT * updatedScalePair.y);
		glScissor(updatedScalePair.z, updatedScalePair.w, WIDTH * updatedScalePair.x, HEIGHT * updatedScalePair.y);
		glEnable(GL_SCISSOR_TEST);
		//glViewport(0, 0, new_width, new_height);
		main_scene.ProjectionMatrix = glm::ortho(-((float)WIDTH / 2) / originalAspect, ((float)WIDTH / 2) / originalAspect, -((float)HEIGHT / 2) / originalAspect, ((float)HEIGHT / 2) / originalAspect, 0.0f, 100.0f);

		tempWidgets = main_scene.getWidgetList();
		tempBodies = main_scene.getBodyList();
		
		//sprintf_s(text2, "x: %d, y: %d", (int)x, (int)y);
		
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && !IS_PRESSED)
		{
			IS_PRESSED = true;
			glfwGetCursorPos(window, &x, &y);
			OnLeftClick(x, y, tempWidgets, tempBodies, new_width, new_height, ++objID, programID, newAspect, widgets);
			std::cout << "Mouse Clicked! Position is, X: " << x << ", Y: " << y << std::endl;
		}
		
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE && IS_PRESSED)
		{
			IS_PRESSED = false;
			std::cout << "Mouse released!" << std::endl;
		}

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS && !IS_RIGHT_PRESSED)
		{
			IS_RIGHT_PRESSED = true;
			glfwGetCursorPos(window, &x, &y);
			OnRightClick(x, y, tempWidgets, tempBodies, new_width, new_height, ++objID, programID, newAspect, widgets);
			std::cout << "Mouse right Clicked! Position is, X: " << x << ", Y: " << y << std::endl;
		}

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE && IS_RIGHT_PRESSED)
		{
			IS_RIGHT_PRESSED = false;
			std::cout << "Mouse released!" << std::endl;
		}

		sprintf_s(numBodies, "%d: Bodies.", tempBodies.size());

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		mPhysicsController.bodies = tempBodies;
		main_scene.setWidgetList(tempWidgets);
		main_scene.setBodiesList(tempBodies);
		//TreeFunc(*tree, tempWidgets);
		//tree = 0;
		//tree->debugRender(true);
		//mPhysicsController.setTree(tree);

		//start = std::chrono::high_resolution_clock::now();
		//mPhysicsController.FixedUpdate();
		mPhysicsController.FixedUpdate_CL(objID + 1, widgets);
		//end = std::chrono::high_resolution_clock::now();
		//float duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		//duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		//std::cout << "Duration of Fixed Update_CL: " << duration << std::endl;
		//sprintf_s(mPhysicsController.physicsTimer, "FixedUpdate: %d", (int)duration);

		//main_scene.renderScene();
		main_scene.renderScene_CL(objID + 1, widgets);
		// Bind our texture in Texture Unit 0
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, Texture);
		// Set our "myTextureSampler" sampler to user Texture Unit 0
		glUniform1i(TextureID, 0);
		printText2D(numBodies, 35, 550, 40);
		//printText2D(text2, 35, 500, 40);
		printText2D(mPhysicsController.physicsTimer, 15, 450, 40);

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

	} // Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0);

	// Cleanup VBO and shader
	//glDeleteBuffers(1, &vertexbuffer);
	//glDeleteBuffers(1, &colourbuffer);
	glDeleteProgram(programID);
	glDeleteVertexArrays(1, &VertexArrayID);
	cleanupText2D();

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	//cudaTest* mCudaObject = new cudaTest();
	//mCudaObject->runCudaTestFunc();
	//mCudaObject->runVectorAdd();
	//delete(mCudaObject);
	//foo mFoo;
	//mFoo.runFooVectorAdd();

	//cudaTest* mCudaObject = new cudaTest();
	//mCudaObject->objectArrayTest();
	//opencl_init();

	std::cout << "Press ENTER to continue...";
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	return 0;
}