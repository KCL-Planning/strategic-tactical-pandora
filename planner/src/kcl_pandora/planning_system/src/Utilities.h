#ifndef KCL_util_functions
#define KCL_util_functions

#include <sstream>

namespace PandoraKCL
{

	/**
	 * Runs external commands
	 */
	std::string runCommand(std::string cmd)
	{
		std::string data;
		FILE *stream;
		char buffer[1000];
		stream = popen(cmd.c_str(), "r");
		while ( fgets(buffer, 1000, stream) != NULL )
			data.append(buffer);
		pclose(stream);
		return data;
	}

	std::string toLowerCase(const std::string& str) {
		std::stringstream ss;	
		int differ = 'A'-'a';
		int ii = str.size();
		for(int i=0; i<ii;i++) {
			char ch = str.at(i);
			if(ch>='A' && ch<='Z')
				ch = ch-differ;
			ss << ch;
		}
		return ss.str();
	}

	std::string convert(double number) {
		std::stringstream ss;
		ss << number;
		return ss.str();
	}
}

#endif
