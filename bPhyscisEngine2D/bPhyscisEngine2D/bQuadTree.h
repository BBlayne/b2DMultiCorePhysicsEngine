#pragma once
#include <stdlib.h>
#include <vector>
#include <iterator>
#include "Dependencies\glm\glm\gtx\transform.hpp"
#include "Dependencies\glm\glm\gtc\matrix_transform.hpp"
#include "bWidget.h"
#include "bRect.h"
#include "bUtils.h"

/// <summary>
/// The QuadTree class object is the overall 'manager' of the quadtree data structure.
/// It's purpose is to create a less resource intensive algorithm for collision detection.
/// A quadtree is a tree data structure in which each internal node has exactly four children. 
/// Quadtrees are most often used to partition a two-dimensional space by recursively 
/// subdividing it into four quadrants or regions. The regions may be square or rectangular, 
/// or may have arbitrary shapes. 
/// They decompose space into adaptable cells
/// Each cell(or bucket) has a maximum capacity. When maximum capacity is reached, the bucket splits
/// The tree directory follows the spatial decomposition of the quadtree.
/// </summary>

enum NODE_DIRECTION {
	NORTH_WEST,
	NORTH_EAST,
	SOUTH_EAST,
	SOUTH_WEST,
	NODE_COUNT
};

class bQuadTree {
private:
	int SCREEN_WIDTH;
	int SCREEN_HEIGHT;
	class bNode;
	bNode* root;
	GLuint programID;
	GLuint MatrixID;
	glm::mat4 MVP; 
	GLuint vertexbuffer;
	GLuint VertexArrayID;
public:
	bQuadTree() {};
	bQuadTree(int pLevel, int _topLeftCornerX, int _topLeftCornerY, int _2DBoundsWidth, int _2DBoundsHeight);
	bQuadTree(int pLevel, glm::vec2 _pos, bRect* bounds);
	~bQuadTree() {};

	bool getRoot(bQuadTree::bNode*& _rootNode) 
	{ 
		_rootNode = (this->root); 
		if (_rootNode != nullptr)
		{
			return true;
		}
		else
		{
			return false;
		}
	};
	bool clearTree();
	// Interfaces
	bool insert(bWidget* _widget);
	bool insert(int _posX, int _posY, int _widthObject, int _heightObject, bWidget* _widget);
	void setBounds(int newWidth, int newHeight)
	{
		this->SCREEN_WIDTH = newWidth;
		this->SCREEN_HEIGHT = newHeight;
	};
	std::vector<bRigidbody2D*> retrieve(std::vector<bRigidbody2D*> &possibleContacts, bWidget* _object);
	void debugRender(bool isRender);
	void debugRenderInit();
	bUtils _util;
};

class bQuadTree::bNode : public bQuadTree {
private:
	glm::vec3 color;
	static const int MAX_CHILDREN = 4;
	static const int MAX_WIDGETS = 16;
	static const int MAX_LEVELS = 5;
	int level;
	int width;
	int height;
	bRect* bounds;
	int upperLeftX;
	int upperLeftY;
	std::vector<bRigidbody2D*> bodies;
	glm::vec3 pos;
	// Children ordered counter clockwise
	bNode* northWest;
	bNode* northEast;
	bNode* southWest;
	bNode* southEast;

	bNode* getNorthWest() { return northWest; };
	bNode* getNorthEast() { return northEast; };
	bNode* getSouthEast() { return southEast; };
	bNode* getSouthWest() { return southWest; };

	int getUpperLeftX() { return this->upperLeftX; };
	int getUpperLeftY() { return this->upperLeftY; };
	int getBoundsWidth() { return this->width; };
	int getBoundsHeight() { return this->height; };

	//bNode** children;
	std::vector<bNode*> vecChildren;

	GLuint debugShaderID;

public:
	bool getChild(NODE_DIRECTION childDirection, bQuadTree::bNode *&childNode);
	//bool getChild(int _index, bQuadTree::bNode *&childNode);
	int getIndex(int _x, int _y, int _width, int _height);
	glm::vec3 getIndex(bWidget* _widget);
	bNode(int _pLevel, int _x, int _y, int _width, int _height);
	bNode(int _pLevel, glm::vec2 pos, bRect* bounds);
	~bNode() {};
	void setColour(glm::vec3 _col) {
		this->color = _col;
	};
	bool clear();
	bool split();
	bool insert(bWidget* _widget);
	bool insert(int _posX, int _posY, int _widthObject, int _heightObject, bWidget* _widget);
	void renderQuad(GLuint _vertexArrayID, GLuint _programID, GLuint _MatrixID, GLuint _vertexbuffer);
	void findLeafs(GLuint _vertexArrayID, GLuint _programID, GLuint _MatrixID, GLuint _vertexbuffer);
	void retrieve(std::vector<bRigidbody2D*> &possibleContacts, bWidget* _object);
	void setOriginalScreenSize(int _parent_width, int _parent_height) { this->SCREEN_WIDTH = _parent_width; this->SCREEN_HEIGHT = _parent_height; };
	glm::mat4 refreshMVP();
};