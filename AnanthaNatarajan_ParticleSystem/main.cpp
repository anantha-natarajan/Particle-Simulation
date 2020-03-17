/*------------------------------------------------------------------------------------------------------------------------------------------------------------

                                                           PARTICLE SIMULATOR
													 Code Written by ANANTHA NATARAJAN
													    Implemented using OpenGl 3

								              -----------------------------------------------------

External Libraries Used:
	*	GLFW for windowing operations
	*	GLM for mathematical operations on vectors and matrices




--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <stack>
#include <cstdlib>
#include <sstream>
#include <fstream>

#define GLEW_STATIC
#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtx/scalar_multiplication.hpp"
#include "ShaderProgram.h"
#include "Camera.h"
#include "State.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#define PI 3.14159265
#define MaxParticles 1000

bool gFullScreen = false;
GLFWwindow *pwindow = NULL;
bool gWireframe = false;
int gWindowWidth = 1280;
int gWindowHeight = 800;
GLFWwindow *GUIwindow = NULL;
int guiWindowWidth = 630;
int guiWindowHeight = 600;

OrbitCamera orbitCam;
float gYaw = 0.0f;
float gPitch = 0.0f;
float gRadius = 1000.0f;
const float MOUSE_SENSITIVITY = 0.25f;

// SIMULATION SETTINGS
float h = 0.0850;
float displayRate = 0.0125f;
float restitution = 0.2f, friction = 0.9f;
float d = 0.3;
int NP = 0;
bool simulation = false;
bool writeFile = false;

std::default_random_engine generator;
glm::vec3 startGenerator, GeneratorDirection;
std::stack <int> deactiveParticleList;
int deactiveCount = MaxParticles;

//This gives a default acceleration to all the particles in the simulation
//glm::vec3 defaultParticleAcceleration = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 defaultParticleAcceleration = glm::vec3(0.0f, -1.0f, 0.0f);
//Giving a WindVelocity will blow the particles in that direction
glm::vec3 windVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 generatorPosition = glm::vec3(0.0f, 5.0f, -100.0f);
glm::vec3 defaultParticleColor = glm::vec3(1.0f, 0.0f, 0.0f);

struct particle {

	particle() {
		position = generatorPosition;
		velocity = glm::vec3(0.0f, 0.0f, 0.0f);
		color = defaultParticleColor;
		acceleration = defaultParticleAcceleration;
		active = false;
		birthTime = 0.0f;
		age = 0.0f;
	}

	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec3 color;
	glm::vec3 acceleration;
	double birthTime;
	double age;
	bool active;
};

particle constrainedParticleList[MaxParticles];

void onKeyPress(GLFWwindow* window, int key, int scancode, int action, int mode);
void glfw_OnFrameBufferSize(GLFWwindow* window, int width, int height);
void glfw_onMouseMove(GLFWwindow* window, double posX, double posY);
bool initOpengl();
double uniRandScalar(double umin, double umax);
double normRandScalar(double mean, double sd);
glm::vec3 vectorGenerator_S();
glm::vec3 vectorGenerator_Du(glm::vec3 w, double maxDisplaceAngle);
glm::vec3 vectorGenerator_Dg(glm::vec3 w, double maxDisplaceAngle);// StandardDeviation = maxDisplaceAngle/3
glm::vec3 positionGenerator_Cu(glm::vec3 center, glm::vec3 surfaceNormal, double R);
glm::vec3 positionGenerator_Cg(glm::vec3 center, glm::vec3 surfaceNormal, double R);// Radius  = 3*StandardDeviation

void GenerateRandomParticle();
bool testAndDeactivate(double currentTime, int particle);
int findDeactiveSpot();

int main() {
	
	if (!initOpengl()) {
		std::cout << "OpenGl initialization failed" << std::endl;
		return -1;
	}
	glm::vec4 clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.00f);
	glm::vec3 normalTriangle, netAcceleration;

	GLfloat particleGenerator[] = {
		0.0f,0.0f,0.0f, 1.0f,0.0f,0.0f
	};
	glm::vec3 VertexA = glm::vec3(-5.0f, 5.0f, -5.0f);
	glm::vec3 VertexB = glm::vec3(5.0f, -5.0f, -5.0f);
	glm::vec3 VertexC = glm::vec3(-5.0f, -5.0f, 5.0f);
	GLfloat triangleVertices[] = {
		VertexA.x,VertexA.y,VertexA.z, 0.0f,1.0f,1.0f,
		VertexB.x,VertexB.y,VertexB.z, 0.0f,1.0f,1.0f,
		VertexC.x,VertexC.y,VertexC.z, 0.0f,1.0f,1.0f
	};
	normalTriangle = glm::cross((VertexC - VertexA), (VertexB - VertexA)) / glm::length(glm::cross((VertexC - VertexA), (VertexB - VertexA)));
	GeneratorDirection = -normalTriangle;
	glm::vec3 translatedVertexA = VertexA + glm::vec3(-10.0f, 0.0f, -100.0f);
	glm::vec3 translatedVertexB = VertexB + glm::vec3(-10.0f, 0.0f, -100.0f);
	glm::vec3 translatedVertexC = VertexC + glm::vec3(-10.0f, 0.0f, -100.0f);

	//To determine the Barycentric coordinates which is used for collision detection
	glm::vec3 vn = glm::cross((translatedVertexB - translatedVertexA), (translatedVertexC - translatedVertexB));
	double A = glm::length(vn) /2;
	double u, v, w;
	glm::vec3 centroid = glm::vec3((translatedVertexA.x + translatedVertexB.x + translatedVertexC.x) / 3, (translatedVertexA.y + translatedVertexB.y + translatedVertexC.y) / 3, (translatedVertexA.z + translatedVertexB.z + translatedVertexC.z) / 3);
	startGenerator = centroid + normalTriangle * 20;

	//INITIALIZING THE VERTEX ARRAY OBJECTS & VERTEX BUFFER OBJECTS for the Traingle and the Particles
	//polygon
	GLuint vboT, vaoT;
	glGenVertexArrays(1, &vaoT);
	glBindVertexArray(vaoT);
	glGenBuffers(1, &vboT);
	glBindBuffer(GL_ARRAY_BUFFER, vboT);
	glBufferData(GL_ARRAY_BUFFER, sizeof(triangleVertices), triangleVertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, NULL);
	glEnableVertexAttribArray(0);
	//color
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, (GLvoid*)(sizeof(GLfloat) * 3));
	glEnableVertexAttribArray(1);

	//particles
	GLuint vboP, vaoP;
	glGenVertexArrays(1, &vaoP);
	glBindVertexArray(vaoP);
	glGenBuffers(1, &vboP);
	glBindBuffer(GL_ARRAY_BUFFER, vboP);
	glBufferData(GL_ARRAY_BUFFER, sizeof(particleGenerator), particleGenerator, GL_DYNAMIC_DRAW);
	//position
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, NULL);
	glEnableVertexAttribArray(0);
	//color
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, (GLvoid*)(sizeof(GLfloat) * 3));
	glEnableVertexAttribArray(1);

	ShaderProgram shaderProgram;
	shaderProgram.loadShader("basic.vert" , "basic.frag");
	glfwSetFramebufferSizeCallback(pwindow, glfw_OnFrameBufferSize);

	glm::mat4 projection;
	glm::vec3 triangleColor = glm::vec3(0.0f, 0.8f, 0.0f);
	glm::vec3 particleColor = glm::vec3(1.0f, 0.0f, 0.0f);
	shaderProgram.use();
	projection = glm::perspective(glm::radians(45.0f), (float)gWindowWidth / (float)gWindowHeight, 0.1f, 500.0f);
	shaderProgram.setUniform("projection", projection);
	shaderProgram.setUniform("part_color", triangleColor);
	glm::mat4 partMatrix = glm::mat4(1.0f);
	
	//Initialize The Deactive Particle List
	for (int i = MaxParticles-1; i >= 0; i--) {
		deactiveParticleList.push(i);
	}

	std::fstream myfile;
	myfile.open("pointCloudAllFrames_Version8.txt", std::ios::out);

	while (!glfwWindowShouldClose(pwindow)) {
		
		glfwMakeContextCurrent(pwindow);
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glfwPollEvents();
		double currentTime = glfwGetTime();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Setting up the Model, View matrix to draw the Triangle on the screen
		glm::mat4 view, polygonModel;
		orbitCam.setLookAt(glm::vec3(0.0f,0.0f,-100.0f));
		orbitCam.rotate(gYaw, gPitch);
		orbitCam.setRadius(gRadius);
		view = orbitCam.getViewMatrix();
		shaderProgram.setUniform("view", view);
		shaderProgram.setUniform("part_color", triangleColor);
		polygonModel = glm::translate(polygonModel, glm::vec3(-10.0f, 0.0f, -100.0f));// *glm::scale(polygonModel, glm::vec3(5, 5, 5));
		shaderProgram.setUniform("model", polygonModel);
		glBindVertexArray(vaoT);
		glDrawArrays(GL_TRIANGLES, 0, 3);
		glBindVertexArray(0);
		
		// SIMULATION LOOP 
		if (simulation) {
			/*if (true) {
				for (int i = 0; i < MaxParticles; i++) {
					myfile << (int)FrameNumber << " " << (double)constrainedParticleList[i].position.x << " " << (double)constrainedParticleList[i].position.y << " " << (double)constrainedParticleList[i].position.z << "\n";
				}
			}
			FrameNumber++;*/

			// GENERATE PARTICLES AS SIMULATION RUNS
			if (NP<MaxParticles) {
				//std::cout << "No of particles " << NP << std::endl;
				for (int j = 0; j < 20; j++) {
					GenerateRandomParticle();
				}
			}

			//SIMULATE FOR EVERY GENERATED PARTICLE
			for (int j = 0; j < MaxParticles;j++) {
				if (constrainedParticleList[j].active == true) {
					// TEST & SIMULATE FOR PARTICLES
					if (testAndDeactivate(glfwGetTime(), j)) {
						// PARTICLE IS DEAD
						NP--;
						continue;
					}else{
						// PARTICLE IS ACTIVE
						// COLLISION DETECTION and RESPONSE
						glm::vec3 newV, newP;
						double tHit;
						glm::vec3 xHit;
						netAcceleration = constrainedParticleList[j].acceleration + (windVelocity - constrainedParticleList[j].velocity) * d;

						// CALCULATING THE NEW VELOCITY AND POSITION AFTER THE TIMESTEP
						newV = constrainedParticleList[j].velocity + netAcceleration * h;
						newP = constrainedParticleList[j].position + constrainedParticleList[j].velocity * h;
		
						// ESTIMATING THE TIME TAKEN TO HIT THE TRIANGLE	
						tHit = glm::dot((translatedVertexA - constrainedParticleList[j].position),normalTriangle)/ glm::dot(constrainedParticleList[j].velocity,normalTriangle);
						if (tHit >= 0 && tHit < h) {
							xHit = constrainedParticleList[j].position + tHit * constrainedParticleList[j].velocity;
							u = glm::dot(glm::cross((translatedVertexB - translatedVertexC), (xHit - translatedVertexC)),normalTriangle) / (2*A);
							v = glm::dot(glm::cross((translatedVertexA - translatedVertexB), (xHit - translatedVertexB)), normalTriangle) / (2 * A);
							w = 1 - u - v;
							if (u >= 0 && v >= 0 && w >= 0) {
								//std::cout << "collision occured in that timestep inside the triangle" << std::endl;
								glm::vec3 newV_AC, newP_AC, newVn, newVt;
								double newD = glm::dot(newP - translatedVertexA, normalTriangle);
								newP_AC = newP - (1 + restitution)*newD*normalTriangle;
								newVn = glm::dot(newV, normalTriangle)*normalTriangle;
								newVt = newV - newVn;
								newV_AC = -restitution * newVn + (1 - friction)*newVt;
								newV = newV_AC;
								newP = newP_AC;
								constrainedParticleList[j].color = glm::vec3(0.0f, 0.0f, 1.0f);
								//Reverse accelertion after collision
								constrainedParticleList[j].acceleration = -constrainedParticleList[j].acceleration;
								//netAcceleration = glm::vec3(0.0f, -0.05f, 0.0f);
							}
						}

						//INTEGRATION - assigning the new position, velocity to the particle
						constrainedParticleList[j].position = newP;
						constrainedParticleList[j].velocity = newV;
						
						//DRAWING PARTICLE IN ITS OWN COLOR
						particleColor = constrainedParticleList[j].color;

						// INITIALISE MODEL MATRIX and DRAW THE PARTICLE 
						glm::mat4 particleModel = glm::translate(partMatrix, newP);
						shaderProgram.setUniform("part_color", particleColor);
						shaderProgram.setUniform("model", particleModel);
						glBindVertexArray(vaoP);
						glDrawArrays(GL_POINTS, 0, 1);
						glBindVertexArray(0);

					}
				}
				}
		}
		glfwSwapBuffers(pwindow);
		
	}
	myfile.close();
	glfwTerminate();
	return 0;
}

bool initOpengl() {

	if (!glfwInit()) {
		std::cout << " GLFW initialization failed" << std::endl;
		return false;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	if (gFullScreen) {
		GLFWmonitor* pMonitor = glfwGetPrimaryMonitor();
		const GLFWvidmode* pVmode = glfwGetVideoMode(pMonitor);
		if (pVmode != NULL) {
			pwindow = glfwCreateWindow(pVmode->width, pVmode->height, "Render Window", pMonitor, NULL);
			//GUIwindow = glfwCreateWindow(pVmode->width, pVmode->height, "GUI Window", pMonitor, NULL);
		}
	}
	else {
		pwindow = glfwCreateWindow(gWindowWidth, gWindowHeight, "Render Window", NULL, NULL);
		//GUIwindow = glfwCreateWindow(guiWindowWidth, guiWindowHeight, "GUI Window", NULL, NULL);
	}
		

	if (pwindow == NULL) {
		std::cout << "Window Creation Failed";
		glfwTerminate();
		return false;
	}

	/*if (GUIwindow == NULL) {
		std::cout << "Window Creation Failed";
		glfwTerminate();
		return false;
	}*/

	glfwMakeContextCurrent(pwindow);
	glfwSetKeyCallback(pwindow, onKeyPress);
	glfwSetCursorPosCallback(pwindow, glfw_onMouseMove);

	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) {
		std::cout << "Glew initialization failed";
		return false;
	}

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glViewport(0, 0, gWindowWidth, gWindowHeight);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
	glEnable(GL_PROGRAM_POINT_SIZE);


	/* Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(GUIwindow, true);
	ImGui_ImplOpenGL3_Init("#version 430 core");
	*/

	return true;
}

double uniRandScalar(double umin, double umax) {
	std::uniform_real_distribution<double> distribution(umin, umax);
	double temp = distribution(generator);
	return temp;
}
double normRandScalar(double mean, double sd) {
	std::normal_distribution<double> distribution(mean, sd);
	double temp = distribution(generator);
	return temp;
}
glm::vec3 vectorGenerator_S() {
	double theta, height, radSph;
	theta = uniRandScalar(-180, 180);
	height = uniRandScalar(-1, 1);
	radSph = sqrt(1-height*height);
	theta = theta * PI / 180.0; //converting degrees to radians
	return glm::vec3(radSph*cos(theta), height, -radSph * sin(theta));
}
glm::vec3 vectorGenerator_Du(glm::vec3 w, double maxDisplaceAngle) {
	glm::vec3 a;
	if (w.y == 0 && w.z == 0) {
		a = glm::vec3(1.0f, 0.0f, 0.0f);
	}
	else {
		a = glm::vec3(0.0f, 1.0f, 0.0f);
	}

	glm::vec3 uz = w;
	glm::vec3 ux = glm::cross(a, uz) / glm::length(glm::cross(a, uz));
	glm::vec3 uy = glm::cross(uz, ux);
	glm::mat3 rotMat = glm::mat3(ux, uy, uz);

	double f = uniRandScalar(0, 1);
	double phi = sqrt(f)*maxDisplaceAngle;
	double theta = uniRandScalar(-180, 180);
	theta = theta * PI / 180.0;
	phi = phi * PI / 180.0;

	glm::vec3 offsetVdash = glm::vec3(cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi));
	glm::vec3 offsetV = rotMat * offsetVdash;
	return offsetV;
}
glm::vec3 vectorGenerator_Dg(glm::vec3 w, double maxDisplaceAngle) {
	// StandardDeviation = maxDisplaceAngle/3
	glm::vec3 a;
	if (w.y == 0 && w.z == 0) {
		a = glm::vec3(1.0f, 0.0f, 0.0f);
	}
	else {
		a = glm::vec3(0.0f, 1.0f, 0.0f);
	}

	glm::vec3 uz = w;
	glm::vec3 ux = glm::cross(a, uz) / glm::length(glm::cross(a, uz));
	glm::vec3 uy = glm::cross(uz, ux);
	glm::mat3 rotMat = glm::mat3(ux, uy, uz);

	double f = normRandScalar(0, maxDisplaceAngle / 3);
	double phi = sqrt(f)*maxDisplaceAngle;
	double theta = uniRandScalar(-180, 180);
	theta = theta * PI / 180.0;
	phi = phi * PI / 180.0;

	glm::vec3 offsetVdash = glm::vec3(cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi));
	glm::vec3 offsetV = rotMat * offsetVdash;
	return offsetV;
}
glm::vec3 positionGenerator_Cu(glm::vec3 center, glm::vec3 surfaceNormal, double R) {

	glm::vec3 a;
	if (surfaceNormal.y == 0 && surfaceNormal.z == 0) {
		a = glm::vec3(1.0f, 0.0f, 0.0f);
	}
	else {
		a = glm::vec3(0.0f, 1.0f, 0.0f);
	}

	glm::vec3 uz = surfaceNormal;
	glm::vec3 ux = glm::cross(a, uz) / glm::length(glm::cross(a, uz));
	glm::vec3 uy = glm::cross(uz, ux);
	glm::mat3 rotMat = glm::mat3(ux, uy, uz);

	double f = uniRandScalar(0, 1);
	double rad = sqrt(f) * R;
	double theta = uniRandScalar(-180, 180);
	theta = theta * PI / 180.0;

	glm::vec3 offsetPdash = glm::vec3(cos(theta)*rad, sin(theta)*rad, 0);
	glm::vec3 offsetP = center + rotMat * offsetPdash;
	return offsetP;

}
glm::vec3 positionGenerator_Cg(glm::vec3 center, glm::vec3 surfaceNormal, double R) {
	// Radius  = 3*StandardDeviation
	glm::vec3 a;
	if (surfaceNormal.y == 0 && surfaceNormal.z == 0) {
		a = glm::vec3(1.0f, 0.0f, 0.0f);
	}
	else {
		a = glm::vec3(0.0f, 1.0f, 0.0f);
	}

	glm::vec3 uz = surfaceNormal;
	glm::vec3 ux = glm::cross(a, uz) / glm::length(glm::cross(a, uz));
	glm::vec3 uy = glm::cross(uz, ux);
	glm::mat3 rotMat = glm::mat3(ux, uy, uz);

	double f = normRandScalar(0, R / 3);
	double rad = sqrt(f) * R;
	double theta = uniRandScalar(-180, 180);
	theta = theta * PI / 180.0;

	glm::vec3 offsetPdash = glm::vec3(cos(theta)*rad, sin(theta)*rad, 0);
	glm::vec3 offsetP = center + rotMat * offsetPdash;
	return offsetP;
}

int findDeactiveSpot() {

	if (deactiveCount != 0) {
		int index = deactiveParticleList.top();
		deactiveParticleList.pop();
		deactiveCount--;
		return index;
	}
	else {
		return -1;
	}
	
}

void GenerateRandomParticle() {
	//find deactivated spot in the array
	//fill in the particle
	int deactiveIndex = findDeactiveSpot();
	if (deactiveIndex != -1) {
		constrainedParticleList[deactiveIndex].velocity = vectorGenerator_Du(GeneratorDirection,30);
		//constrainedParticleList[deactiveIndex].velocity = vectorGenerator_Dg(GeneratorDirection, 10);
		//constrainedParticleList[deactiveIndex].velocity = vectorGenerator_S();
		constrainedParticleList[deactiveIndex].velocity = constrainedParticleList[deactiveIndex].velocity * 10;
		constrainedParticleList[deactiveIndex].position = startGenerator;
		constrainedParticleList[deactiveIndex].active = true;
		constrainedParticleList[deactiveIndex].birthTime = glfwGetTime();
		constrainedParticleList[deactiveIndex].acceleration = defaultParticleAcceleration;
		//constrainedParticleList[deactiveIndex].color = vectorGenerator_S();
		constrainedParticleList[deactiveIndex].age = uniRandScalar(3, 8);
		//constrainedParticleList[deactiveIndex].age = 3;
		NP++;
	}
	else {
		//std::cout << "No of particles " << NP << std::endl;
		//std::cout << "Not enough space" << std::endl;
	}

}

bool testAndDeactivate(double currentTime, int particle) {
	bool status;
	if (currentTime - constrainedParticleList[particle].birthTime > constrainedParticleList[particle].age) {
		deactiveParticleList.push(particle);
		deactiveCount++;
		constrainedParticleList[particle].active = false;
		constrainedParticleList[particle].position = generatorPosition;
		constrainedParticleList[particle].velocity = glm::vec3(0.0f, 0.0f, 0.0f);
		constrainedParticleList[particle].birthTime = 0.0f;
		constrainedParticleList[particle].color = defaultParticleColor;
		status = true;
	}
	else {
		status = false;
	}
	
	return status;
}

void onKeyPress(GLFWwindow* window, int key, int scancode, int action, int mode) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window,GL_TRUE);

	if (key == GLFW_KEY_W && action == GLFW_PRESS) {
		gWireframe = !gWireframe;
		if (gWireframe) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		else {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
	}

	if (key == GLFW_KEY_S && action == GLFW_PRESS) {
		simulation = true;
	}

	if (key == GLFW_KEY_W && action == GLFW_RELEASE) {
		writeFile = true;
	}

}

void glfw_onMouseMove(GLFWwindow* window, double posX, double posY)
{
	static glm::vec2 lastMousePos = glm::vec2(0, 0);

	if (glfwGetMouseButton(pwindow, GLFW_MOUSE_BUTTON_LEFT) == 1) {
		//std::cout << "here  " << posX << "--" << posY << std::endl;
		gYaw -= ((float)posX - lastMousePos.x) * MOUSE_SENSITIVITY;
		gPitch += ((float)posY - lastMousePos.y) * MOUSE_SENSITIVITY;
		//std::cout << "here  " << gYaw << "--" << gPitch << std::endl;
	}

	if (glfwGetMouseButton(pwindow, GLFW_MOUSE_BUTTON_RIGHT) == 1) {
		float dx = 10.0f * ((float)posX - lastMousePos.x);
		float dy = 10.0f * ((float)posY - lastMousePos.y);

		//std::cout << "here  " << dx << "--" << dy << std::endl;

		
	}

	lastMousePos.x = (float)posX;
	lastMousePos.y = (float)posY;

}

void glfw_OnFrameBufferSize(GLFWwindow* window, int width, int height)
{
	gWindowWidth = width;
	gWindowHeight = height;
	glViewport(0, 0, gWindowWidth, gWindowHeight);
}
