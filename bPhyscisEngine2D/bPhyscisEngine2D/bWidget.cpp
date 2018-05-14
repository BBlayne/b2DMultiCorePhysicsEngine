#include "bWidget.h"

bWidget::bWidget(Shape* myShape, Transform* myTransform, bRigidbody2D* myRigidBody2D)
{
	this->myRigidBody2D = myRigidBody2D;
	this->myRigidBody2D->shape = myShape;
	this->myRigidBody2D->setParent(this);
	this->myTransform = myTransform;
	this->myShader = new bShader();
}

bWidget::bWidget(Shape* myShape, Transform* myTransform, bRigidbody2D* myRigidBody2D, bShader* myShader)
{
	this->myRigidBody2D = myRigidBody2D;
	this->myRigidBody2D->shape = myShape;
	this->myRigidBody2D->setParent(this);
	this->myTransform = myTransform;
	this->myShader = myShader;
}

bWidget::bWidget(Shape* myShape, Transform* _transform, bRigidbody2D* myRigidBody2D, bShader* myShader, Mesh* mesh, int _id)
{

	this->myRigidBody2D = myRigidBody2D;
	this->myRigidBody2D->shape = myShape;
	this->myRigidBody2D->transform = _transform;
	this->myRigidBody2D->setParent(this);
	this->myTransform = _transform;
	this->myTransform->mBody = this->myRigidBody2D;
	this->myShader = myShader;
	this->mesh = mesh;
	this->objectID = _id;
}

void bWidget::SetWidgetID(int myID)
{
	this->objectID = myID;
}

int bWidget::getWidgetID()
{
	return this->objectID;
}

void bWidget::InitBuffers()
{
	glGenBuffers(1, myShader->getUVBuffer());
	glBindBuffer(GL_ARRAY_BUFFER, *myShader->getUVBuffer());
	glBufferData(GL_ARRAY_BUFFER, this->mesh->UVPts.size() * sizeof(float), &this->mesh->UVPts[0], GL_STATIC_DRAW);

	glGenBuffers(1, myShader->getVertexBuffer());
	glBindBuffer(GL_ARRAY_BUFFER, *myShader->getVertexBuffer());
	glBufferData(GL_ARRAY_BUFFER, mesh->vertex_points.size() * sizeof(float), &this->mesh->vertex_points[0], GL_STATIC_DRAW);
}

void bWidget::render()
{
		// Create and compile our GLSL program from the shaders


	// Get a handle for our "myTextureSampler" uniform
	//myShader->setTexID(glGetUniformLocation(myShader->getProgramID(), "myTextureSampler"));

	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, myShader->getTex());

	// Set our "myTextureSampler" sampler to user Texture Unit 0
	glUniform1i(myShader->getTexID(), 0);

	GLuint randomColour = glGetUniformLocation(myShader->getProgramID(), "mColour");

	//glm::vec3 randomColourVector = glm::vec3((rand() % 255 + 1) / 255, (rand() % 255 + 1) / 255, (rand() % 255 + 1) / 255);
	glm::vec3 randomColourVector = glm::vec3(0.5, 0.75, 0.9);
	glUniform3f(randomColour, randomColourVector.x, randomColourVector.y, randomColourVector.z);

	// Get a handle for our "MVP" uniform
	myShader->setMatrixID(glGetUniformLocation(myShader->getProgramID(), "MVP"));

	// Use our shader
	glUseProgram(myShader->getProgramID());

	// Send our transformation to the currently bound shader, 
	// in the "MVP" uniform
	glUniformMatrix4fv(myShader->getMatrixID(), 1, GL_FALSE, &myShader->getMVP()[0][0]);
	
	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, *myShader->getVertexBuffer());
	glVertexAttribPointer(
		0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);

	// 2nd attribute buffer : UVs
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, *myShader->getUVBuffer());
	glVertexAttribPointer(
		1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		2,                                // size : U+V => 2
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
		);

	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, this->mesh->vertex_points.size());

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	
}