
#include "ShaderProgram.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include "glm/gtc/type_ptr.hpp"

ShaderProgram::ShaderProgram()
	: mHandle(0)
{
}

ShaderProgram::~ShaderProgram()
{
	glDeleteProgram(mHandle);
}

bool ShaderProgram::loadShader(const char* vsFilename, const char* fsFilename)
{

	string vShaderSrc = fileToString(vsFilename);
	string fShaderSrc = fileToString(fsFilename);

	const GLchar* vsSourcePtr = vShaderSrc.c_str();
	const GLchar* fsSourcePtr = fShaderSrc.c_str();

	GLuint vertexShader, fragmentShader;

	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(vertexShader, 1, &vsSourcePtr, NULL);
	glShaderSource(fragmentShader, 1, &fsSourcePtr, NULL);

	glCompileShader(vertexShader);
	glCompileShader(fragmentShader);

	checkCompileErrors(vertexShader, VERTEX);
	checkCompileErrors(fragmentShader, FRAGMENT);

	mHandle = glCreateProgram();
	glAttachShader(mHandle, vertexShader);
	glAttachShader(mHandle, fragmentShader);
	glLinkProgram(mHandle);

	checkCompileErrors(mHandle, PROGRAM);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	mUniformLocations.clear();

	return true;
}

void ShaderProgram::use() {
	if (mHandle > 0) {
		glUseProgram(mHandle);
	}
}

string ShaderProgram::fileToString(const string& filename)
{
	std::stringstream ss;
	std::ifstream file;

	try {
		file.open(filename, std::ios::in);

		if (!file.fail()) {
			ss << file.rdbuf();
		}
		
		file.close();
	}

	catch(std::exception exp)
	{
		std::cerr << "Error reading shader filename! " << std::endl;
	}

	return ss.str();
}

void ShaderProgram::checkCompileErrors(GLuint shader, ShaderType type)
{

	GLint result;

	if (type == PROGRAM) {

		glGetProgramiv(shader, GL_COMPILE_STATUS, &result);

		if (!result) {
			GLint length = 0;
			glGetProgramiv(shader, GL_INFO_LOG_LENGTH, &length);

			string infoLog(length, ' ');
			glGetProgramInfoLog(shader, length, NULL, &infoLog[0]);

			std::cout << "Error! Program failed to link " << infoLog[0] << std::endl;
		}

	}
	else // vertex or fragment 
	{
		glGetShaderiv(shader, GL_COMPILE_STATUS, &result);

		if (!result) {
			GLint length = 0;
			glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);

			string infoLog(length, ' ');
			glGetShaderInfoLog(shader, length, NULL, &infoLog[0]);

			std::cout << "Error! Shader failed to compile " << infoLog[0] << std::endl;
		}

	}

}

GLint ShaderProgram::getUniformLocation(const GLchar* name)
{
	std::map<string, GLint>::iterator it = mUniformLocations.find(name);

	if (it == mUniformLocations.end()) {
		mUniformLocations[name] = glGetUniformLocation(mHandle, name);
	}

	return mUniformLocations[name];
}

void ShaderProgram::setUniform(const GLchar* name, const glm::vec2& v)
{
	GLint loc = getUniformLocation(name);
	glUniform2f(loc, v.x, v.y);
}

void ShaderProgram::setUniform(const GLchar* name, const glm::vec3& v) 
{
	GLint loc = getUniformLocation(name);
	glUniform3f(loc, v.x, v.y, v.z);
}

void ShaderProgram::setUniform(const GLchar* name, const glm::vec4& v)
{
	GLint loc = getUniformLocation(name);
	glUniform4f(loc, v.x, v.y, v.z, v.w);
}

void ShaderProgram::setUniform(const GLchar* name, const glm::mat4& m)
{
	GLint loc = getUniformLocation(name);
	glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(m));
}