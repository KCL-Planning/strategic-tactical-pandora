#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include "dpengine/shaders/ShaderParser.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <istream>
#include <iterator>

namespace DreadedPE
{

ShaderParser* ShaderParser::instance_ = NULL;

ShaderParser& ShaderParser::getInstance()
{
	if (instance_ == NULL)
	{
		instance_ = new ShaderParser();
	}
	return *instance_;
}

ShaderParser::ShaderParser()
{

}

std::string ShaderParser::getSource(const std::string& relative_path)
{
/*
#ifdef _WIN32
		std::stringstream ss;
		ss << "Get the source: " << relative_path << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
*/
	// Check if we have already parsed this shader:
	if (parsed_shaders_.count(relative_path) != 0)
	{
/*
#ifdef _WIN32
		std::stringstream ss;
		ss << "Return already parsed shader: " << relative_path << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
*/
		return parsed_shaders_[relative_path];
	}

	// Otherwise we parse the shader.
	std::stringstream source;
	std::set<std::string> closed_list;
	if (parse(relative_path, source, closed_list))
	{
		return source.str();
	}

	std::cerr << "Parser error!" << std::endl;

#ifdef _WIN32
	OutputDebugString("Parser error!");
#endif
	return std::string();
}

bool ShaderParser::parse(const std::string& relative_path, std::stringstream& results, const std::set<std::string>& closed_list)
{
	// Check if we have this result cached:
	if (parsed_shaders_.count(relative_path) != 0)
	{
		std::string s = parsed_shaders_[relative_path];
		results << s;
/*
#ifdef _WIN32
		std::stringstream ss;
		ss << "Use the preloaded result: " << std::endl << s << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
*/	
		return true;
	}

	std::ifstream shader_file(relative_path.c_str());

	if (!shader_file.good())
	{
/*
#ifdef _WIN32
		std::stringstream ss;
		ss << "Could not load shader: " << relative_path << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
*/
		std::cerr << "Could not load shader: " << relative_path << std::endl;
		return false;
	}

	//std::string stringBuffer(std::istreambuf_iterator<char>(fileIn), (std::istreambuf_iterator<char>()));
	std::string line;
	std::stringstream local_ss;
	while (std::getline(shader_file, line))
	{
		std::vector<std::string> tokens;
		std::istringstream iss(line);
		std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(tokens));

		if (tokens.empty())
		{
			continue;
		}

		if ("#include" == tokens[0] && tokens.size() == 2)
		{
			std::cout << "Inlude the file: " << tokens[1] << std::endl;
/*
#ifdef _WIN32
			std::stringstream ss;
			ss << "Include the file: " << tokens[1] << std::endl;
			OutputDebugString(ss.str().c_str());
#endif
*/
			// Only include the file if it has not been processed in this sequence (i.e. we are in an infinite loop).
			if (closed_list.count(tokens[1]) != 0)
			{
				std::cerr << "We are in an infinite loop! The file: " << tokens[1] << " has already been processed!" << std::endl;
#ifdef _WIN32
				std::stringstream ss;
				ss << "We are in an infinite loop! The file: " << tokens[1] << " has already been processed!" << std::endl;
				OutputDebugString(ss.str().c_str());
#endif
				return false;
			}
			std::set<std::string> new_closed_list(closed_list);
			new_closed_list.insert(tokens[1]);
			if (!parse(tokens[1], local_ss, new_closed_list))
			{
				return false;
			}
/*
#ifdef _WIN32
			{
			std::stringstream ss;
			ss << "Result after include: " << local_ss.str() << std::endl;
			OutputDebugString(ss.str().c_str());
			}
#endif
*/
		}
		else
		{
/*
#ifdef _WIN32
			std::stringstream ss;
			ss << "Add the line: " << line << std::endl;
			OutputDebugString(ss.str().c_str());
#endif
*/
			local_ss << line << std::endl;
		}
	}

	parsed_shaders_.insert(std::make_pair(relative_path, local_ss.str()));
	results << local_ss.str();
/*
#ifdef _WIN32
			std::stringstream ss;
			ss << "Final result: " << results.str() << std::endl;
			OutputDebugString(ss.str().c_str());
#endif
*/
	return true;
}

};
