#ifndef CORE_LOADERS_WAFEFRONT_LOADER_H
#define CORE_LOADERS_WAFEFRONT_LOADER_H

#include <string>
#include <vector>

class Shape;

struct Face
{
	Face(unsigned int vertex_index, unsigned int texture_coordinate_index, unsigned int normal_index)
		: vertex_index_(vertex_index), texture_coordinate_index_(texture_coordinate_index), normal_index_(normal_index)
	{
	
	}

	Face(const std::string& string);

	unsigned int vertex_index_, texture_coordinate_index_, normal_index_;
};

struct Triangle
{
	Triangle(const Face& f1, const Face& f2, const Face& f3)
	{
		faces_.push_back(&f1);
		faces_.push_back(&f2);
		faces_.push_back(&f3);
	}

	~Triangle()
	{
		for (std::vector<const Face*>::const_iterator ci = faces_.begin(); ci != faces_.end(); ++ci)
		{
			delete *ci;
		}
	}

	std::vector<const Face*> faces_;
};

std::ostream& operator<<(std::ostream& os, const Triangle& triangle);

class WavefrontLoader
{
public:

	static Shape* importShape(const std::string& filename);

private:

};

#endif
