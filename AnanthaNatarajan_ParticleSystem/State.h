#ifndef STATE_H
#define STATE_H
#include "glm/glm.hpp"

class State
{
public:
	State();
	~State();
	
	float time;
	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec3 acceleration;
	
	void printState();
};

#endif