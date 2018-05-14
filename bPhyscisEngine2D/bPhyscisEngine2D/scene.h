#pragma once
#include "Dependencies\glm\glm\glm.hpp"
#include <vector>
#include <list>
#include "bWidget.h"

class scene {
private:
	std::vector<bWidget*> listWidgetsScene;
	std::vector<bRigidbody2D*> listBodiesScene;
protected:

public:
	float aspect;
	glm::mat4 ViewMatrix;
	glm::mat4 ProjectionMatrix;
	void setupScene(std::vector<bWidget*> simulationObjects);
	void updateScene(std::vector<bWidget*> simulationObjects);
	bWidget* getObjectAtIndex(int index);
	void addObjectToScene(bWidget* simulation_object);
	void renderScene();
	void instantiateObject(Shape* shape);
	
	//
	void setWidgetList(std::vector<bWidget*> _listWidgetsScene) { this->listWidgetsScene = _listWidgetsScene; };
	std::vector<bWidget*> getWidgetList() { return this->listWidgetsScene; };
	void setBodiesList(std::vector<bRigidbody2D*> _listBodiesScene) { this->listBodiesScene = _listBodiesScene; };
	std::vector<bRigidbody2D*> getBodyList() { return this->listBodiesScene; };
	void renderScene_CL(int numWidgets, widget* _widgets);
};
