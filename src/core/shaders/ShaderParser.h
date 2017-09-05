#ifndef CORE_SHADERS_SHADER_PARSER_H
#define CORE_SHADERS_SHADER_PARSER_H

#include <map>
#include <string>
#include <set>
#include <sstream>

/**
 * Shaders can have include files, this is not supported by OpenGL but it makes
 * my life a lot easier by not having to replicate the same code accross multiple
 * shaders. The new tag "include <relative path>" is parsed by this parser. The
 * result is stored inside a hash table.
 */
class ShaderParser
{
public:
	static ShaderParser& getInstance();

	std::string getSource(const std::string& relative_path);
private:

	static ShaderParser* instance_;

	ShaderParser();

	bool parse(const std::string& relative_path, std::stringstream& result, const std::set<std::string>& closed_list);

	std::map<std::string, std::string> parsed_shaders_;
};

#endif
