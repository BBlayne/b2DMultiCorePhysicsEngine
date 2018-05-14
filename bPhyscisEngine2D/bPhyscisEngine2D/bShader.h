#pragma once
#include "bGLOBALS.h"
#include <string>
#include "Dependencies\GL\freeglut.h"
#include "Dependencies\glm\glm\glm.hpp"
#include "Dependencies\glm\glm\gtx\transform.hpp"
#include "Dependencies\glm\glm\gtc\matrix_transform.hpp"
#include "Dependencies\Texture\texture.hpp"
#include "Transform.h"

class bShader {
private:
	GLuint vertexbuffer;
	GLuint uvbuffer;
	GLuint programID;
	GLuint TextureID;
	GLuint Texture;
	GLuint MatrixID;
	GLuint colourbuffer;
	glm::mat4 projection;
	glm::mat4 camera;
	glm::mat4 model;
	glm::mat4 MVP;
public:

	glm::mat4 refreshMVP(glm::mat4 ProjectionMatrix, glm::mat4 ViewMatrix, glm::mat4 rotation, glm::mat4 scale, glm::vec3 position);
	glm::mat4 refreshMVP(Transform& _transform, glm::mat4 ProjectionMatrix, glm::mat4 ViewMatrix);
	glm::mat4 refreshMVP(glm::vec3 position, glm::mat4 rotation, glm::mat4 scale, glm::mat4 ProjectionMatrix, glm::mat4 ViewMatrix);
	std::string shader_name;

	bShader();
	bShader(int screen_width, int screen_height);
	~bShader() { };

	// setters & getters
	void setVertexBuffer(GLuint _vertexbuf) { this->vertexbuffer = _vertexbuf; };
	GLuint* getVertexBuffer() { return &this->vertexbuffer; };
	void setUVBuffer(GLuint _uvbuffer) { this->uvbuffer = _uvbuffer; };
	GLuint* getUVBuffer() { return &this->uvbuffer; };
	void setProgramID(GLuint _programID) { this->programID = _programID; };
	GLuint getProgramID() { return this->programID; };
	void setTexID(GLuint _texID) { this->TextureID = _texID; };
	GLuint getTexID() { return this->TextureID; };
	void setTex(GLuint _tex) { this->Texture = _tex; };
	GLuint getTex() { return this->Texture; };
	void setColourBuffer(GLuint _colourBuf) { this->colourbuffer = _colourBuf; };
	GLuint* getColourBuffer() { return &this->colourbuffer; };
	void setMatrixID(GLuint _matID) { this->MatrixID = _matID; };
	GLuint getMatrixID() { return this->MatrixID; };
	void setMVP(glm::mat4 _mvp) { this->MVP = _mvp; };
	glm::mat4 getMVP() { return this->MVP; };
	void LoadTextureDDS(const char * imagepath);

	void setProjection(glm::mat4 _projection) { this->projection = _projection; };
	glm::mat4 getProjection() { return this->projection; };
	void setCamera(glm::mat4 _view) { this->camera = _view; };
	glm::mat4 getCamera() { return this->camera; };
	void setModel(glm::mat4 _model) { this->model = _model; };
	glm::mat4 getModel() { return this->model; };


};