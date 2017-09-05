#include "glslshader.h"

GLSLProgram* GLSLProgram::last_used_shader_ = NULL;

GLSLProgram::GLSLProgram(const string& vertexShader)
{
	GLSLShader vertex_shader(GL_VERTEX_SHADER, vertexShader);
	shaders_.push_back(vertex_shader);
}

GLSLProgram::GLSLProgram(const string& vertexShader, const string& fragmentShader, bool is_transform_feedback_shader)
{
	GLSLShader vertex_shader(GL_VERTEX_SHADER, vertexShader);
	shaders_.push_back(vertex_shader);
	if (is_transform_feedback_shader)
	{
		GLSLShader fragment_shader(GL_GEOMETRY_SHADER, fragmentShader);
		shaders_.push_back(fragment_shader);
	}
	else
	{
		GLSLShader fragment_shader(GL_FRAGMENT_SHADER, fragmentShader);
		shaders_.push_back(fragment_shader);
	}
}

GLSLProgram::GLSLProgram(const string& vertexShader, const string& geometrytShader, const string& fragmentShader)
{
	GLSLShader vertex_shader(GL_VERTEX_SHADER, vertexShader);
	shaders_.push_back(vertex_shader);
	GLSLShader geometry_shader(GL_GEOMETRY_SHADER, geometrytShader);
	shaders_.push_back(geometry_shader);
	GLSLShader fragment_shader(GL_FRAGMENT_SHADER, fragmentShader);
	shaders_.push_back(fragment_shader);
}

void GLSLProgram::addShader(GLuint type, const string& file)
{
	GLSLShader shader(type, file);
	shaders_.push_back(shader);
}

GLSLProgram::~GLSLProgram()
{

}

void GLSLProgram::unload()
{
	for (std::vector<GLSLShader>::const_iterator ci = shaders_.begin(); ci != shaders_.end(); ++ci)
	{
		glDetachShader(m_programID, (*ci).id_);
		glDeleteShader((*ci).id_);
	}
    glDeleteShader(m_programID);
}

bool GLSLProgram::initialize()
{
    m_programID = glCreateProgram();

	for (std::vector<GLSLShader>::iterator ci = shaders_.begin(); ci != shaders_.end(); ++ci)
	{
		GLSLShader& shader = *ci;
		shader.id_ = glCreateShader(shader.type_);
		shader.source_ = readFile(shader.filename_);

		if (shader.source_.empty() || shader.id_ == 0)
		{
#ifdef _WIN32
			{
			std::stringstream ss;
			ss << "could not find " << shader.filename_ << std::endl;
			std::cerr << ss.str().c_str() << std::endl;
			OutputDebugString(ss.str().c_str());
			}
#else
			std::stringstream ss;
			ss << "could not find " << shader.filename_ << std::endl;
			std::cerr << ss.str().c_str() << std::endl;
#endif
			return false;
		}

		const GLchar* tmp = static_cast<const GLchar*>(shader.source_.c_str());
		glShaderSource(shader.id_, 1, (const GLchar**)&tmp, NULL);

		if (!compileShader(shader))
		{
#ifdef _WIN32
			OutputDebugString("Could not compile the shaders, they are invalid");
#else
			std::cerr << "Could not compile the shaders, they are invalid" << std::endl;
			std::cerr << shader.source_ << std::endl;
#endif
			return false;
		}

		glAttachShader(m_programID, shader.id_);
	}
	glLinkProgram(m_programID);
	return true;
}

void GLSLProgram::linkProgram()
{
	glLinkProgram(m_programID);

	// Check if the linking was successful.
	GLint is_linked;
	glGetProgramiv(m_programID, GL_LINK_STATUS, &is_linked);
	if (is_linked != GL_TRUE)
	{
		GLsizei log_length;
		GLchar buffer[1024];
		glGetProgramInfoLog(m_programID, 1024, &log_length, &buffer[0]);
#ifdef _WIN32
		// Print any messages.
		OutputDebugString(buffer);
#else
		std::string tmp(buffer);
		std::cerr << tmp << std::endl;
#endif
		exit(1);
	}
}

GLuint GLSLProgram::getUniformLocation(const string& name)
{
	map<string, GLuint>::iterator i = m_uniformMap.find(name);
	if (i == m_uniformMap.end())
	{
		GLuint location = glGetUniformLocation(m_programID, name.c_str());
			
		if (location == -1)
		{
/*
#ifdef _WIN32
			std::stringstream ss;
			ss << "Could not find the location of " << name;
			for (std::vector<GLSLShader>::iterator ci = shaders_.begin(); ci != shaders_.end(); ++ci)
			{
				GLSLShader& shader = *ci;
				ss << " in " << shader.filename_ << "(" << shader.id_ << "); ";
			}
			MessageBox(NULL, ss.str().c_str(), "Error", MB_OK);
#endif
			*/
			std::cerr << "Could not find the uniform name of: " << name << std::endl;
		}
		
		m_uniformMap.insert(std::make_pair(name, location));
		return location;
	}
	return (*i).second;
}

GLuint GLSLProgram::getAttribLocation(const string& name)
{
    map<string, GLuint>::iterator i = m_attribMap.find(name);
    if (i == m_attribMap.end())
    {
        GLuint location = glGetAttribLocation(m_programID, name.c_str());
        m_attribMap.insert(std::make_pair(name, location));

		if (location == -1)
		{

#ifdef _WIN32
			std::stringstream ss;
			ss << "Could not find the location of " << name;
			for (std::vector<GLSLShader>::const_iterator ci = shaders_.begin(); ci != shaders_.end(); ++ci)
			{
				ss << (*ci).filename_.c_str() << " ";
			}
			MessageBox(NULL, ss.str().c_str(), "Error", MB_OK);
#endif

			std::cerr << "Could not find the attribute " << name << std::endl;
			exit(1);
		}

        return location;
    }

    return (*i).second;
}

void GLSLProgram::sendUniform(const string& name, const int id)
{
    GLuint location = getUniformLocation(name);
    glUniform1i(location, id);
}

void GLSLProgram::sendUniform(const string& name, GLuint id)
{
    GLuint location = getUniformLocation(name);
    glUniform1i(location, id);
}

void GLSLProgram::sendUniform4x4(const string& name, const float* matrix, unsigned int values, bool transpose)
{
    GLuint location = getUniformLocation(name);
    glUniformMatrix4fv(location, values, transpose, matrix);
}

void GLSLProgram::sendUniform3x3(const string& name, const float* matrix, unsigned int values, bool transpose)
{
    GLuint location = getUniformLocation(name);
    glUniformMatrix3fv(location, values, transpose, matrix);
}

void GLSLProgram::sendUniform(const string& name, const float red, const float green, const float blue, const float alpha)
{
    GLuint location = getUniformLocation(name);
    glUniform4f(location, red, green, blue, alpha);
}

void GLSLProgram::sendUniform3i(const string& name, const int& array, unsigned int values)
{
    GLuint location = getUniformLocation(name);
    glUniform3iv(location, values, &array);
}

void GLSLProgram::sendUniform4i(const string& name, const int& array, unsigned int values)
{
    GLuint location = getUniformLocation(name);
    glUniform4iv(location, values, &array);
}

void GLSLProgram::sendUniform3f(const string& name, const float& array, unsigned int values)
{
    GLuint location = getUniformLocation(name);
    glUniform3fv(location, values, &array);
}

void GLSLProgram::sendUniform4f(const string& name, const float& array, unsigned int values)
{
    GLuint location = getUniformLocation(name);
    glUniform4fv(location, values, &array);
}

void GLSLProgram::sendUniform(const string& name, const float x, const float y,
                    const float z)
{
    GLuint location = getUniformLocation(name);
    glUniform3f(location, x, y, z);
}

void GLSLProgram::sendUniform(const string& name, const float scalar)
{
    GLuint location = getUniformLocation(name);
    glUniform1f(location, scalar);
}

void GLSLProgram::sendUniform(const string& name, const float& array, unsigned int values)
{
	GLuint location = getUniformLocation(name);
	glUniform1fv(location, values, &array);
}

void GLSLProgram::sendUniform(const string& name, const int& array, unsigned int values)
{
	GLuint location = getUniformLocation(name);
	glUniform1iv(location, values, &array);
}

void GLSLProgram::bindAttrib(unsigned int index, const string& attribName)
{
    glBindAttribLocation(m_programID, index, attribName.c_str());
}

void GLSLProgram::bindShader()
{
    glUseProgram(m_programID);
	last_used_shader_ = this;
}

string GLSLProgram::readFile(const string& filename)
{
	ShaderParser& parser = ShaderParser::getInstance();
	return parser.getSource(filename);
	/*
	ifstream fileIn(filename.c_str());

	if (!fileIn.good())
	{
		std::cerr << "Could not load shader: " << filename << std::endl;
		return string();
	}

	string stringBuffer(std::istreambuf_iterator<char>(fileIn), (std::istreambuf_iterator<char>()));
	return stringBuffer;
	*/
}

bool GLSLProgram::compileShader(const GLSLShader& shader)
{
    glCompileShader(shader.id_);
    GLint result = 0xDEADBEEF;
    glGetShaderiv(shader.id_, GL_COMPILE_STATUS, &result);

    if (!result)
    {
        std::cout << "Could not compile shader: " << shader.id_ << std::endl;
        outputShaderLog(shader);
        return false;
    }

    return true;
}

void GLSLProgram::outputShaderLog(const GLSLShader& shader)
{
    //vector<char> infoLog;
    GLint infoLen;
	glGetShaderiv(shader.id_, GL_INFO_LOG_LENGTH, &infoLen);
	//infoLog.resize(infoLen);
	char* infolog = new char[infoLen + 1];

    std::cerr << "GLSL Shader: Shader contains errors, please validate this shader!" << std::endl;
    glGetShaderInfoLog(shader.id_, infoLen, &infoLen, infolog);

    std::cerr << infolog << std::endl;
#ifdef _WIN32
	std::stringstream ss;
	ss << "File: " << shader.filename_ << std::endl;
	ss << infolog << std::endl;
    MessageBox(NULL, ss.str().c_str(), "Error", MB_OK);
#endif
	std::ofstream file("shader errors.txt");
	file << string(infolog) << std::endl;
	file.close();
}
