#ifndef GLSL_SHADER_H_INCLUDED
#define GLSL_SHADER_H_INCLUDED

#ifdef _WIN32
#include <windows.h>
#endif

#include <map>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include "GL/glew.h"

#include "ShaderParser.h"

class Shape;

using std::string;
using std::ifstream;
using std::map;
using std::vector;

class GLSLProgram
{
public:
    struct GLSLShader
    {
		GLSLShader(GLuint type, const string& filename)
			: type_(type), filename_(filename)
		{

		}
		 
		GLuint type_;
        unsigned int id_;
        string filename_;
        string source_;
    };

	GLSLProgram(const string& vertexShader);
	GLSLProgram(const string& vertexShader, const string& fragmentShader, bool is_transform_feedback_shader = false);
	GLSLProgram(const string& vertexShader, const string& geometrytShader, const string& fragmentShader);
	void addShader(GLuint type, const string& file);
    virtual ~GLSLProgram();
    void unload();
    bool initialize();
	void linkProgram();
    GLuint getUniformLocation(const string& name);
    GLuint getAttribLocation(const string& name);
    void sendUniform(const string& name, const int id);
	void sendUniform(const string& name, GLuint id);
    void sendUniform4x4(const string& name, const float* matrix, unsigned int values = 1, bool transpose=false);
    void sendUniform3x3(const string& name, const float* matrix, unsigned int values = 1, bool transpose=false);
    void sendUniform(const string& name, const float red, const float green, const float blue, const float alpha);
	void sendUniform3i(const string& name, const int& array, unsigned int values);
	void sendUniform4i(const string& name, const int& array, unsigned int values);
	void sendUniform3f(const string& name, const float& array, unsigned int values);
	void sendUniform4f(const string& name, const float& array, unsigned int values);
    void sendUniform(const string& name, const float x, const float y, const float z);
    void sendUniform(const string& name, const float scalar);
	void sendUniform(const string& name, const float& array, unsigned int values);
	void sendUniform(const string& name, const int& array, unsigned int values);
    void bindAttrib(unsigned int index, const string& attribName);
    void bindShader();

protected:
	std::map<const Shape*, GLuint> shape_to_vbo_;

private:
	string readFile(const string& filename);
    bool compileShader(const GLSLShader& shader);
    void outputShaderLog(const GLSLShader& shader);

protected:
	unsigned int m_programID;

	static GLSLProgram* last_used_shader_;

private:
	std::vector<GLSLShader> shaders_;

    map<string, GLuint> m_uniformMap;
    map<string, GLuint> m_attribMap;
};

#endif 