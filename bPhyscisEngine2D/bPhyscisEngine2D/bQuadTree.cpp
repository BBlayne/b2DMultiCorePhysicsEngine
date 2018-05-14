#include "bQuadTree.h"

bQuadTree::bNode::bNode(int pLevel, int _x, int _y, int _width, int _height)
{
	this->level = pLevel;
	this->width = _width;
	this->bodies.clear();
	this->height = _height;
	this->upperLeftX = _x;
	this->upperLeftY = _y;
	this->pos = glm::vec3(_x + _width / 2, _y + _height / 2, 0);

	//debugShaderID = LoadShaders("debugVert.vert", "debugFrag.frag");
}


bQuadTree::bQuadTree(int pLevel, int _x, int _y, int _width, int _height)
{
	this->SCREEN_WIDTH = _width;
	this->SCREEN_HEIGHT = _height;
	this->root = new bNode(-1, _x, _y, _width, _height);
	this->root->setOriginalScreenSize(this->SCREEN_WIDTH, this->SCREEN_HEIGHT);
	this->root->setColour(glm::vec3(1, 1, 1));
	//this->root->setBounds(0, 0);
	this->root->split();
}

bQuadTree::bQuadTree(int pLevel, glm::vec2 topLeft, bRect* bounds)
{
	this->SCREEN_WIDTH = bounds->getWidth();
	this->SCREEN_HEIGHT = bounds->getHeight();
	this->root = new bNode(-1, topLeft.x, topLeft.y, bounds->getWidth(), bounds->getHeight());
	this->root->setOriginalScreenSize(this->SCREEN_WIDTH, this->SCREEN_HEIGHT);
	this->root->setColour(glm::vec3(1, 1, 1));
	this->root->split();
}

/// <summary>
/// Fetches a specific child node from the current node in the tree.
/// </summary>
/*
bool bQuadTree::bNode::getChild(int index, bQuadTree::bNode *&childNode)
{
	if (this->children[index] != nullptr)
	{
		childNode = (this->children[index]);
		return true;
	}
	else
	{
		return false;
	}
}*/

bool bQuadTree::bNode::getChild(NODE_DIRECTION childDirection, bQuadTree::bNode *&childNode)
{
	switch (childDirection)
	{
	case NODE_DIRECTION::NORTH_EAST:
		childNode = (this->getNorthEast());
		return true;
	case NODE_DIRECTION::NORTH_WEST:
		childNode = (this->getNorthWest());
		return true;
	case NODE_DIRECTION::SOUTH_EAST:
		childNode = (this->getSouthEast());
		return true;
	case NODE_DIRECTION::SOUTH_WEST:
		childNode = (this->getSouthWest());
		return true;
	default:
		return false;
	}
}

bool bQuadTree::clearTree()
{
	bNode* temp_root = 0;
	if (this->getRoot(temp_root))
	{
		temp_root->clear();
		temp_root->split();
	}
	else
	{
		return false;
	}
	return true;
}

bool bQuadTree::bNode::clear()
{
	if (this->getNorthEast() != nullptr)
	{
		this->getNorthEast()->clear();
		this->getNorthEast()->bodies.clear();
		delete(northEast);
		this->northEast = 0;
	}
	if (this->getNorthWest() != nullptr)
	{
		this->getNorthWest()->clear();
		this->getNorthWest()->bodies.clear();
		delete(northWest);
		this->northWest = 0;
	}


	if (this->getSouthWest() != nullptr)
	{
		this->getSouthWest()->clear();
		this->getSouthWest()->bodies.clear();
		delete(southWest);
		this->southWest = 0;
	}

	if (this->getSouthEast() != nullptr)
	{
		this->getSouthEast()->clear();
		this->getSouthEast()->bodies.clear();
		delete(southEast);
		this->southEast = 0;
	}
	this->bodies.clear();

	return true;
}

bool bQuadTree::bNode::split()
{
	int subwidth = this->width / 2;
	int subheight = this->height / 2;
	this->northEast = new bNode(this->level + 1, upperLeftX + subwidth, upperLeftY, subwidth, subheight);
	this->northEast->setOriginalScreenSize(this->SCREEN_WIDTH, this->SCREEN_HEIGHT);
	this->northEast->setColour(glm::vec3(1, 0, 0));
	this->northWest = new bNode(this->level + 1, upperLeftX, upperLeftY, subwidth, subheight);
	this->northWest->setOriginalScreenSize(this->SCREEN_WIDTH, this->SCREEN_HEIGHT);
	this->northWest->setColour(glm::vec3(0, 1, 0));
	this->southWest = new bNode(this->level + 1, upperLeftX, upperLeftY + subheight, subwidth, subheight);
	this->southWest->setOriginalScreenSize(this->SCREEN_WIDTH, this->SCREEN_HEIGHT);
	this->southWest->setColour(glm::vec3(1, 1, 0));
	this->southEast = new bNode(this->level + 1, upperLeftX + subwidth, upperLeftY + subheight, subwidth, subheight);
	this->southEast->setOriginalScreenSize(this->SCREEN_WIDTH, this->SCREEN_HEIGHT);
	this->southEast->setColour(glm::vec3(1, 1, 1));

	return true;
}

int bQuadTree::bNode::getIndex(int _x, int _y, int _width, int _height)
{
	int index = -1;
	double horizontalMidpoint = this->getUpperLeftX() + (this->getBoundsWidth() / 2);
	double verticalMidpoint = this->getUpperLeftY() + (this->getBoundsHeight() / 2);
	// Object can completely fit within the top quadrants
	bool topQuadrant = ((_y < verticalMidpoint) && (_y + _height < verticalMidpoint));
	// Object can completely fit within the bottom quadrants
	bool bottomQuadrant = (_y > verticalMidpoint);

	if ((_x < horizontalMidpoint) && (_x + _width < horizontalMidpoint))
	{
		if (topQuadrant)
		{
			index = 1;
		}
		else if (bottomQuadrant)
		{
			index = 2;
		}
	}
	// Object can completely fit within the right quadrants
	else if (_x > horizontalMidpoint)
	{
		if (topQuadrant)
		{
			index = 0;
		}
		else if (bottomQuadrant)
		{
			index = 3;
		}
	}

	return index;
}

glm::vec3 bQuadTree::bNode::getIndex(bWidget* _widget)
{
	//std::cout << this->getBoundsWidth() / 2 << std::endl;
	if (this->getBoundsWidth() <= 32)
	{
			int x = 0;
	}
	glm::vec3 xany = _util.HalfOffsetScreenToScreenCoords(_widget->myTransform->position.x, _widget->myTransform->position.y, this->SCREEN_WIDTH, this->SCREEN_HEIGHT);
	int _halfwidth  = _widget->myRigidBody2D->shape->GetExtents().x / 2;
	int _halfheight = _widget->myRigidBody2D->shape->GetExtents().y / 2;
	int _y = xany.y;
	int _x = xany.x;

	//int index = -1;
	glm::vec3 index = glm::vec3(1, -1, 0);
	double horizontalMidpoint = this->getUpperLeftX() + (this->getBoundsWidth() / 2);
	double verticalMidpoint = this->getUpperLeftY() + (this->getBoundsHeight() / 2);
	// Object can completely fit within the top quadrants
	bool topQuadrant = false;
	bool bottomQuadrant = false;
	bool bothQuadrants = false;

	if ((_y <= verticalMidpoint) && (_y + _halfheight >= verticalMidpoint) ||
		(_y >= verticalMidpoint) && (_y - _halfheight <= verticalMidpoint))
	{
		bothQuadrants = true;
	}
	else if ((_y < verticalMidpoint) && (_y + _halfheight < verticalMidpoint))
		topQuadrant = true;	
	// Object can completely fit within the bottom quadrants
	else if ((_y > verticalMidpoint) && (_y - _halfheight > verticalMidpoint))
		bottomQuadrant = true;


		


	// So on reflection there is a LARGE oversight in this algorithm, namely that when determining which bin
	// an object should be inserted into, it doesn't consider the case of OVERLAP by objects over multiple bins.
	// So instead of specifically solving for a boundary overlapping all bins, we should consider a general
	// solution for all objects possibly overlaping two horizontal, two vertical, or all possible combinations
	// of overlap.
	// First, we need to consider the CONSTRAINED cases of when the object perfectly fits.
	if ((_x < horizontalMidpoint) && ((_x + _halfwidth) < horizontalMidpoint))
	{
		// Does the object perfectly fit in the Western quadrant?
		if (topQuadrant)
		{
			index = glm::vec3(1, 1, 0);
		}
		else if (bottomQuadrant)
		{
			index = glm::vec3(1, 2, 0);
		}
		else if (bothQuadrants)
		{
			// if per chance the overlap also spans north/south
			if (_y < verticalMidpoint)
			{
				index = glm::vec3(2, 1, 2);
			}
			else
			{
				index = glm::vec3(2, 2, 1);
			}
		}
	}
	else if ((_x > horizontalMidpoint) && (horizontalMidpoint < (_x - _halfwidth)))
	{
		// Does the object perfectly fit in the eastern quadrant?
		if (topQuadrant)
		{
			index = glm::vec3(1, 0, 0);
		}
		else if (bottomQuadrant)
		{
			index = glm::vec3(1, 3, 0);
		}
		else if (bothQuadrants)
		{
			// if per chance the overlap also spans north/south
			if (_y < verticalMidpoint)
			{
				index = glm::vec3(2, 0, 3);
			}
			else
			{
				index = glm::vec3(2, 3, 0);
			}
		}
	}
	else if ((_x <= horizontalMidpoint) && ((_x + _halfwidth) >= horizontalMidpoint))
	{
		// Now we have an overlap situation, doesn't matter which way overlaps more.
		// Best to check the niche case here.
		if (bothQuadrants)
		{
			// if per chance the overlap also spans north/south
			if (_y < verticalMidpoint)
			{
				index = glm::vec3(4, 1, 2);
			}
			else
			{
				index = glm::vec3(4, 2, 1);
			}
		}
		else if (topQuadrant)
		{
			index = glm::vec3(2, 1, 0);
		}
		else if (bottomQuadrant)
		{
			index = glm::vec3(2, 2, 3);
		}
	}
	else if ((_x >= horizontalMidpoint) && ((_x - _halfwidth) <= horizontalMidpoint))
	{
		// Now we have an overlap situation, doesn't matter which way overlaps more.
		// Best to check the niche case here.
		if (bothQuadrants)
		{
			// if per chance the overlap also spans north/south
			if (_y < verticalMidpoint)
			{
				index = glm::vec3(4, 3, 0);
			}
			else
			{
				index = glm::vec3(4, 0, 3);
			}
		}
		else if (topQuadrant)
		{
			index = glm::vec3(2, 1, 0);
		}
		else if (bottomQuadrant)
		{
			index = glm::vec3(2, 2, 3);
		}
	}

	return index;
}

bool bQuadTree::insert(bWidget* _widget)
{
	bNode* temp_root = 0;
	if (this->getRoot(temp_root))
	{
		temp_root->insert(_widget);
	}
	else
	{
		return false;
	}

	return true;
}

bool bQuadTree::insert(int _posX, int _posY, int _widthObject, int _heightObject, bWidget* _widget)
{
	bNode* _root = 0;
	if (this->getRoot(_root))
	{
		_root->insert(_posX, _posY, _widthObject, _heightObject, _widget);
	}
	else
	{
		return false;
	}

	return true;
}

bool bQuadTree::bNode::insert(int _posX, int _posY, int _objWidth, int _objHeight, bWidget* _widget)
{
	if (this->getNorthEast() != nullptr)
	{
		int index = this->getIndex(_posX, _posY, _objWidth, _objHeight);

		if (index != -1)
		{
			switch (index)
			{
			case 0:
				this->northEast->insert(_posX, _posY, _objWidth, _objHeight, _widget);
				break;
			case 1:
				this->northWest->insert(_posX, _posY, _objWidth, _objHeight, _widget);
				break;
			case 2:
				this->southWest->insert(_posX, _posY, _objWidth, _objHeight, _widget);
				break;
			case 3:
				this->southEast->insert(_posX, _posY, _objWidth, _objHeight, _widget);
				break;
			}
		}
	}

	bodies.push_back(_widget->myRigidBody2D);

	if (bodies.size() > this->MAX_WIDGETS && this->level < this->MAX_LEVELS)
	{
		if (this->getNorthEast() != nullptr)
		{
			this->split();
		}

		int i = 0;
		while (i < bodies.size())
		{
			glm::vec3 xany = _util.HalfOffsetScreenToScreenCoords(bodies[i]->getParent()->myTransform->position.x, bodies[i]->getParent()->myTransform->position.y,
				this->SCREEN_WIDTH, this->SCREEN_HEIGHT);
			int _x = xany.x;
			int _y = xany.y;
			glm::vec2 _bounds = bodies[i]->shape->GetExtents();
			
			int index = this->getIndex(_x, _y, _bounds.x, _bounds.y);
			if (index != -1)
			{
				
				switch (index)
				{
				case 0:
					this->northEast->insert(_posX, _posY, _objWidth, _objHeight, _widget);
					break;
				case 1:
					this->northWest->insert(_posX, _posY, _objWidth, _objHeight, _widget);
					break;
				case 2:
					this->southWest->insert(_posX, _posY, _objWidth, _objHeight, _widget);
					break;
				case 3:
					this->southEast->insert(_posX, _posY, _objWidth, _objHeight, _widget);
					break;
				}
				bodies.erase(bodies.begin() + i);
			}
			else
			{
				i++;
			}
		}
	}

	return true;
}

bool bQuadTree::bNode::insert(bWidget* _widget)
{
	if (this->getNorthEast() != nullptr)
	{
		glm::vec3 index = this->getIndex(_widget);

		if (index.y != -1)
		{
			if (index.x < 2)
			{
				switch ((int)index.y)
				{
				case 0:
					this->northEast->insert(_widget);
					break;
				case 1:
					this->northWest->insert(_widget);
					break;
				case 2:
					this->southWest->insert(_widget);
					break;
				case 3:
					this->southEast->insert(_widget);
					break;
				}
			}
			else if ((int)index.x < 3)
			{
				switch ((int)index.y)
				{
				case 0:
					this->northEast->insert(_widget);
					break;
				case 1:
					this->northWest->insert(_widget);
					break;
				case 2:
					this->southWest->insert(_widget);
					break;
				case 3:
					this->southEast->insert(_widget);
					break;
				}

				switch ((int)index.z)
				{
				case 0:
					this->northEast->insert(_widget);
					break;
				case 1:
					this->northWest->insert(_widget);
					break;
				case 2:
					this->southWest->insert(_widget);
					break;
				case 3:
					this->southEast->insert(_widget);
					break;
				}
			}
			else
			{
				this->northEast->insert(_widget);
				this->northWest->insert(_widget);
				this->southWest->insert(_widget);
				this->southEast->insert(_widget);
			}


			return true;
		}
	}

	bodies.push_back(_widget->myRigidBody2D);
	int mx_wdgts = this->MAX_WIDGETS;

	if (bodies.size() > this->MAX_WIDGETS && this->level < this->MAX_LEVELS)
	{
		if (this->getNorthEast() == nullptr)
		{
			this->split();
		}

		int i = 0;
		while (i < bodies.size())
		{

			glm::vec3 index = this->getIndex(bodies[i]->getParent());
			if (index.y != -1)
			{
				bRigidbody2D* temp = bodies[i];
				bodies.erase(bodies.begin() + i);
				if (index.x < 2)
				{
					switch ((int)index.y)
					{
					case 0:
						this->northEast->insert(temp->getParent());
						break;
					case 1:
						this->northWest->insert(temp->getParent());
						break;
					case 2:
						this->southWest->insert(temp->getParent());
						break;
					case 3:
						this->southEast->insert(temp->getParent());
						break;
					}
				}
				else if ((int)index.x < 3)
				{
					switch ((int)index.y)
					{
					case 0:
						this->northEast->insert(temp->getParent());
						break;
					case 1:
						this->northWest->insert(temp->getParent());
						break;
					case 2:
						this->southWest->insert(temp->getParent());
						break;
					case 3:
						this->southEast->insert(temp->getParent());
						break;
					}

					switch ((int)index.z)
					{
					case 0:
						this->northEast->insert(temp->getParent());
						break;
					case 1:
						this->northWest->insert(temp->getParent());
						break;
					case 2:
						this->southWest->insert(temp->getParent());
						break;
					case 3:
						this->southEast->insert(temp->getParent());
						break;
					}
				}
				else
				{
					this->northEast->insert(_widget);
					this->northWest->insert(_widget);
					this->southWest->insert(_widget);
					this->southEast->insert(_widget);
				}
			}
			else
			{
				i++;
			}
		}
	}

	return true;
}

void bQuadTree::bNode::renderQuad(GLuint _vertexArrayID, GLuint _programID, GLuint _MatrixID, GLuint _vertexbuffer )
{
	//GLuint VertexArrayID;
	//glGenVertexArrays(1, &VertexArrayID);
	//glBindVertexArray(VertexArrayID);
	//debugShaderID = LoadShaders("debugVert.vert", "debugFrag.frag");
	GLuint programID = _programID; //LoadShaders("debugVert.vert", "debugFrag.frag");

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_square_buffer_data), g_square_buffer_data, GL_STATIC_DRAW);


	//glBindVertexArray(_vertexArrayID);
	// Use our shader
	glUseProgram(programID);
	// Send our transformation to the currently bound shader, 
	// in the "MVP" uniform
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &this->refreshMVP()[0][0]);
	GLuint randomColour = glGetUniformLocation(programID, "mColour");

	//glm::vec3 randomColourVector = glm::vec3((rand() % 255 + 1) / 255, (rand() % 255 + 1) / 255, (rand() % 255 + 1) / 255);
	glm::vec3 randomColourVector = this->color;
	glUniform3f(randomColour, randomColourVector.x, randomColourVector.y, randomColourVector.z);
	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glVertexAttribPointer(
		0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);

	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, 2 * 3); // 12*3 indices starting at 0 -> 12 triangles

	glDisableVertexAttribArray(0);
	//glBindVertexArray(0);
}

void bQuadTree::bNode::findLeafs(GLuint _vertexArrayID, GLuint _programID, GLuint _MatrixID, GLuint _vertexbuffer)
{
	if (this->getNorthEast() != nullptr)
	{
		/*
		bNode* temp = 0;
		for (int i = 0; i < MAX_CHILDREN; i++)
		{
			if (this->getChild(i, temp))
			{
				temp->findLeafs(_vertexArrayID, _programID, _MatrixID, _vertexbuffer);
			}				
		}*/

		this->northEast->findLeafs(_vertexArrayID, _programID, _MatrixID, _vertexbuffer);
		this->northWest->findLeafs(_vertexArrayID, _programID, _MatrixID, _vertexbuffer);
		this->southWest->findLeafs(_vertexArrayID, _programID, _MatrixID, _vertexbuffer);
		this->southEast->findLeafs(_vertexArrayID, _programID, _MatrixID, _vertexbuffer);
	}
	else
	{
		this->renderQuad(_vertexArrayID, _programID, _MatrixID, _vertexbuffer);
	}
	
}

std::vector<bRigidbody2D*> bQuadTree::retrieve(std::vector<bRigidbody2D*> &_possibleContacts, bWidget* _object)
{
	bNode* _root = 0;
	if (this->getRoot(_root))
	{
		_root->retrieve(_possibleContacts, _object);
	}

	return _possibleContacts;
}

void bQuadTree::bNode::retrieve(std::vector<bRigidbody2D*> &_possibleContacts, bWidget* _object)
{
	glm::vec3 index = this->getIndex(_object);
	if (index.y != -1 && this->getNorthEast() != nullptr)
	{
		if ((int)index.x < 3)
		{
			switch ((int)index.y)
			{
			case 0:
				this->northEast->retrieve(_possibleContacts, _object);
				break;
			case 1:
				this->northWest->retrieve(_possibleContacts, _object);
				break;
			case 2:
				this->southWest->retrieve(_possibleContacts, _object);
				break;
			case 3:
				this->southEast->retrieve(_possibleContacts, _object);
				break;
			}
		}
		else
		{
			switch ((int)index.y)
			{
			case 0:
				this->northEast->retrieve(_possibleContacts, _object);
				break;
			case 1:
				this->northWest->retrieve(_possibleContacts, _object);
				break;
			case 2:
				this->southWest->retrieve(_possibleContacts, _object);
				break;
			case 3:
				this->southEast->retrieve(_possibleContacts, _object);
				break;
			}
		}

	}

	// add all objects in "bodies" vector to the vector we are passing back up the tree.
	//std::move(bodies.begin(), bodies.end(), std::back_inserter(_possibleContacts));
	_possibleContacts.reserve(bodies.size() + _possibleContacts.size());
	_possibleContacts.insert(_possibleContacts.end(), bodies.begin(), bodies.end());
}

void bQuadTree::debugRenderInit()
{

	//GLuint VertexArrayID;
	//glGenVertexArrays(1, &VertexArrayID);
	//glBindVertexArray(VertexArrayID);
	//debugShaderID = LoadShaders("debugVert.vert", "debugFrag.frag");
	GLuint programID = LoadShaders("debugVert.vert", "debugFrag.frag");
	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	// MVP
	glm::mat4 projectionMatrix = glm::ortho(-this->SCREEN_WIDTH / 2.0f, this->SCREEN_WIDTH / 2.0f,
		-this->SCREEN_HEIGHT / 2.0f, this->SCREEN_HEIGHT / 2.0f, -1.0f, 100.0f);


	glm::mat4 camera = glm::lookAt(
		glm::vec3(0, 0, 1), // Camera is at (0,0,1), in World Space
		glm::vec3(0, 0, 0), // and looks at the origin
		glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
		);

	glm::mat4 ModelMatrixWithTranslationMatrix = glm::translate(glm::vec3(0,0,0)) * glm::mat4(1.0f) * glm::mat4(1.0f);
	glm::mat4 MVP = projectionMatrix * camera * ModelMatrixWithTranslationMatrix;

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_square_buffer_data), g_square_buffer_data, GL_STATIC_DRAW);

	this->MVP = MVP;
	this->vertexbuffer = vertexbuffer;
	this->VertexArrayID = VertexArrayID;
	this->programID = programID;
	this->MatrixID = MatrixID;

	//glBindVertexArray(0);
}

void bQuadTree::debugRender(bool isRender)
{
	if (isRender)
	{
		bNode* _root = 0;
		if (this->getRoot(_root))
		{
			_root->findLeafs(this->VertexArrayID, this->programID, this->MatrixID, this->vertexbuffer);
			//_root->renderQuad(this->VertexArrayID, this->programID, this->MatrixID, this->vertexbuffer);
		}
	}
	else
	{
		delete(this);
	}

}

glm::mat4 bQuadTree::bNode::refreshMVP()
{

	float defaultScale = 385;
	int mLevel = this->level;
	float aspect = (float)this->SCREEN_WIDTH / this->SCREEN_HEIGHT;

	float scaling = (1 / glm::pow(4, (mLevel + 1))) * defaultScale * (mLevel + 1);
	// MVP
	glm::mat4 projectionMatrix = glm::ortho(-this->SCREEN_WIDTH / 2.0f, this->SCREEN_WIDTH / 2.0f,
		-this->SCREEN_HEIGHT / 2.0f, this->SCREEN_HEIGHT / 2.0f, -1.0f, 100.0f);


	glm::mat4 camera = glm::lookAt(
		glm::vec3(0, 0, 1), // Camera is at (0,0,1), in World Space
		glm::vec3(0, 0, 0), // and looks at the origin
		glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
		);



	glm::mat4 scaleMat = glm::mat4(1.0f) * glm::scale(glm::vec3(scaling * aspect, scaling, 1.0f));

	glm::vec3 _position = _util.ScreenToWorldSpaceCoords(this->pos, this->SCREEN_WIDTH, this->SCREEN_HEIGHT);
	//std::cout << _position.x << ", " << _position.y << std::endl;

	glm::mat4 ModelMatrixWithTranslationMatrix = glm::translate(_position) * glm::mat4(1.0f) * scaleMat;
	glm::mat4 MVP = projectionMatrix * camera * ModelMatrixWithTranslationMatrix;

	return MVP;
}