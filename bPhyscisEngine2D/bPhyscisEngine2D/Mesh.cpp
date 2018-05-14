#include "Mesh.h"

Mesh::Mesh()
{

}

Mesh::Mesh(const GLfloat* _vertices, const GLfloat* _uvs)
{
	for (int i = 0; i < G_SQR_UV_BUF_DATA_SIZE; i++)
	{
		this->UVPts.push_back(_uvs[i]);
	}

	for (int i = 0; i < G_SQR_UV_BUF_DATA_SIZE; i += 2)
	{
		this->UVs.push_back(glm::vec2(_uvs[i], _uvs[i + 1]));
	}

	for (int i = 0; i < G_SQR_BUF_DATA_SIZE; i++)
	{
		this->vertex_points.push_back(_vertices[i]);
	}

	for (int i = 0; i < G_SQR_BUF_DATA_SIZE; i++)
	{
		this->vertex_points.push_back(_vertices[i]);
	}

	setVertices(this->vertex_points);
}

Mesh::Mesh(std::vector<glm::vec2> UVs, std::vector<float> UVPts, std::vector<float> vec_vertex_points, std::vector<float> vec_vertex_points_2)
{
	this->UVs = UVs;
	this->UVPts = UVPts;
	this->vertex_points = vec_vertex_points;
	this->setVertices(vec_vertex_points_2);
}

void Mesh::setVertices(std::vector<float> vertex_pts)
{
	glm::vec3 vertex;
	for (int i = 0; i < vertex_pts.size(); i += 3)
	{
		vertex.x = vertex_pts.at(i);
		vertex.y = vertex_pts.at(i+1);
		vertex.z = vertex_pts.at(i+2);
		vertices.push_back(vertex);
	}
}