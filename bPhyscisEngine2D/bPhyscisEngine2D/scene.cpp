#include "scene.h"

void scene::setupScene(std::vector<bWidget*> simulationObjects)
{
	this->listWidgetsScene = simulationObjects;
}

void scene::updateScene(std::vector<bWidget*> simulationObjects)
{
	this->listWidgetsScene = simulationObjects;
}

bWidget* scene::getObjectAtIndex(int index)
{
	return this->listWidgetsScene.at(index);
}

void scene::addObjectToScene(bWidget* simulation_object)
{
	this->listWidgetsScene.push_back(simulation_object);
}

void scene::renderScene()
{
	for (int i = 0; i < this->listWidgetsScene.size(); i++)
	{
		this->listWidgetsScene.at(i)->myShader->refreshMVP(*listWidgetsScene.at(i)->myTransform, ProjectionMatrix, ViewMatrix);
		this->listWidgetsScene.at(i)->render();
	}
}

void scene::renderScene_CL(int numWidgets, widget* _widgets)
{
	for (int i = 0; i < numWidgets; i++)
	{
		glm::vec3 pos = glm::vec3(_widgets[i].t_cl.position_cl.x, _widgets[i].t_cl.position_cl.y, _widgets[i].t_cl.position_cl.z);
		this->listWidgetsScene.at(i)->myTransform->position = pos;
		this->listWidgetsScene.at(i)->myShader->refreshMVP(*listWidgetsScene.at(i)->myTransform, ProjectionMatrix, ViewMatrix);
		this->listWidgetsScene.at(i)->render();
	}
}