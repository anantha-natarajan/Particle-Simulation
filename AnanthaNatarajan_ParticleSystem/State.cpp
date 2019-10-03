#include "State.h"
#include <iostream>
#include <vector>


State::State()
	:position(0.0f,0.0f,0.0f),
	velocity(0.0f,0.0f,0.0f),
	acceleration(0.0f,0.0f,0.0f),
	time(0)
{
}


State::~State()
{
}

void State::printState() {

	std::cout << " The time is t " << time << std::endl;
	std::cout << " The new velocity is " << velocity.x << " + " << velocity.y << " + " << velocity.z << std::endl;
	std::cout << " The new position is " << position.x << " + " << position.y << " + " << position.z << std::endl;

}