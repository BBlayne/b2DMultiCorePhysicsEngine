#pragma once
#include <vector>
#include "Dependencies\GL\glew.h"
#include "Dependencies\glm\glm\glm.hpp"
#include "bGLOBALS.h"

class Mesh {
public:
	std::vector<float> vertex_points;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> normals;
	std::vector<glm::vec2> UVs; // circle
	std::vector<float> UVPts; // circle

	void setVertices(std::vector<float> vertex_points);
	GLuint vertexbuffer;
	GLuint uvbuffer;
	GLuint programID;
	GLuint TextureID;
	GLuint Texture;
	GLuint MatrixID;
	GLuint colourbuffer;

	Mesh();
	Mesh(const GLfloat* _vertices, const GLfloat* _uvs);
	Mesh(std::vector<glm::vec2> UVs, std::vector<float> UVPts, std::vector<float> vertex_points, std::vector<float> vecSqrPts);
	~Mesh() {};
};