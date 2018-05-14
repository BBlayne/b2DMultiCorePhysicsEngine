#include "bShader.h"

bShader::bShader()
{
	this->setProjection(glm::mat4(1.0f));
	this->setCamera(glm::mat4(1.0f));
	this->setModel(glm::mat4(1.0f));
	this->setMVP(getProjection() * getCamera() * getModel());
}

bShader::bShader(int screen_width, int screen_height)
{
	glm::mat4 projectionMatrix = glm::ortho(-screen_width / 2.0f, screen_width / 2.0f,
		-screen_height / 2.0f, screen_height / 2.0f, -1.0f, 100.0f);

	this->setProjection(projectionMatrix);

	glm::mat4 camera = glm::lookAt(
		glm::vec3(0, 0, 1), // Camera is at (0,0,1), in World Space
		glm::vec3(0, 0, 0), // and looks at the origin
		glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
		);

	this->setCamera(camera);
	this->setModel(glm::mat4(1.0f));

	setMVP(projectionMatrix * camera * getModel());
}

glm::mat4 bShader::refreshMVP(glm::mat4 ProjectionMatrix, glm::mat4 ViewMatrix, glm::mat4 rotation, glm::mat4 scale, 
	glm::vec3 position)
{

	glm::mat4 ModelMatrixWithTranslationMatrix = glm::translate(position) * rotation * scale;

	setMVP(ProjectionMatrix * ViewMatrix * ModelMatrixWithTranslationMatrix);
	return ProjectionMatrix * ViewMatrix * ModelMatrixWithTranslationMatrix;
}

glm::mat4 bShader::refreshMVP(Transform& _transform, glm::mat4 ProjectionMatrix, glm::mat4 ViewMatrix)
{

	glm::mat4 ModelMatrixWithTranslationMatrix = glm::translate(_transform.position) * _transform.rotation * (_transform.scale);


	setMVP(ProjectionMatrix * ViewMatrix * ModelMatrixWithTranslationMatrix);
	return ProjectionMatrix * ViewMatrix * ModelMatrixWithTranslationMatrix;
}

glm::mat4 bShader::refreshMVP(glm::vec3 position, glm::mat4 rotation, glm::mat4 scale, glm::mat4 ProjectionMatrix, glm::mat4 ViewMatrix)
{

	

	glm::mat4 ModelMatrixWithTranslationMatrix = glm::translate(position) * rotation * (scale);


	setMVP(ProjectionMatrix * ViewMatrix * ModelMatrixWithTranslationMatrix);
	return ProjectionMatrix * ViewMatrix * ModelMatrixWithTranslationMatrix;
}

void bShader::LoadTextureDDS(const char* imagepath)
{
	this->setTex(loadDDS(imagepath));
}