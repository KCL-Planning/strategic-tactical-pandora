#include "PipeNetwork.h"

#include "Structure.h"
#include "../level/MissionSite.h"
#include "../ontology/InspectionPoint.h"
#include "../../../core/scene/SceneManager.h"
#include "../../../core/texture/TargaTexture.h"
#include "../../../core/scene/Material.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../core/shaders/LineShader.h"
#include "../../../shapes/Cube.h"
#include "../../../shapes/Line.h"
#include <queue>

float PipeNetworkNode::pipe_direction_cost[] = { 1, sqrt(2), 1, sqrt(2), 1, sqrt(2), 1, sqrt(2) };
glm::vec2 PipeNetworkNode::pipe_direction[] = { glm::vec2(0, 1), glm::vec2(1, 1), glm::vec2(1, 0), glm::vec2(1, -1), glm::vec2(0, -1), glm::vec2(-1, -1), glm::vec2(-1, 0), glm::vec2(-1, 1) };

PipeNetwork::PipeNetwork(SceneManager& scene_manager, SceneNode* parent, const std::vector< MissionSite* >& mission_sites, float grid_size)
	: Entity(scene_manager, parent, glm::translate(glm::mat4(1.0f), glm::vec3(0, 1, 0)), OBSTACLE, "Pipenetwork")
{
	// Establish the size of the area we want to cover in piping.
	float min_x = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();

	float min_y = std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	
	std::vector<glm::vec3> source_destination_points;
	
	for (std::vector<MissionSite*>::const_iterator ci = mission_sites.begin(); ci != mission_sites.end(); ++ci)
	{
		MissionSite* mission_site = *ci;
		for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
		{
			Structure* structure = *ci;
			for (std::vector<InspectionPoint*>::const_iterator ci = structure->getInspectionPoints().begin(); ci != structure->getInspectionPoints().end(); ++ci)
			{
				InspectionPoint* inspection_point = *ci;
				const glm::vec3& p = inspection_point->getVisiblePoint();
				
				std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
				
				if (min_x > p.x) min_x = p.x;
				if (max_x < p.x) max_x = p.x;
				
				if (min_y > p.z) min_y = p.z;
				if (max_y < p.z) max_y = p.z;
				
				source_destination_points.push_back(p);
			}
		}
	}
	
	// Extend the boundaries slightly to avoid deadends.
	min_x = (min_x - 5) / grid_size;
	max_x = (max_x + 5) / grid_size;
	min_y = (min_y - 5) / grid_size;
	max_y = (max_y + 5) / grid_size;
	
	std::cout << "Grid dimensions (" << min_x << ", " << max_x << ") -- (" << min_y << ", " << max_y << ")" << std::endl;
	
	int grid_width_ = max_x - min_x;
	int grid_height_ = max_y - min_y;
	
	// Initialise the grid.
	std::vector<int> grid;
	grid.resize((max_x - min_x) * (max_y - min_y), 0);
	
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	
	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.7f, 0.7f, 0.7f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(1.0f, 1.0f, 0.0f, 1.0f);

	Material* terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add2DTexture(*grass_texture);
	
	Texture* eroded_metal_texture = TargaTexture::loadTexture("data/textures/eroded_metal.tga");
	MaterialLightProperty* eroded_metal_ambient = new MaterialLightProperty(0.7f, 0.7f, 0.7f, 1.0f);
	MaterialLightProperty* eroded_metal_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* eroded_metal_specular = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* eroded_metal_emmisive = new MaterialLightProperty(0.7f, 0.7f, 0.7f, 1.0f);

	pipe_material_ = new Material(*eroded_metal_ambient, *eroded_metal_diffuse, *eroded_metal_specular, *eroded_metal_emmisive);
	pipe_material_->add2DTexture(*eroded_metal_texture);
	
	
	//Cube* cube = new Cube(grid_size, grid_size, grid_size);
	
	std::cout << "Initialise grid." << std::endl;
	// Find out which grid cells are actually free and which ones are blocked by obstacles.
	for (int i = 0; i < grid.size(); ++i)
	{
		glm::vec3 mid_point((i % grid_width_) * grid_size + min_x, 2.0f, (i / grid_width_) * grid_size + min_y);
		
		if (scene_manager.getRoot().doesCollide(NULL, mid_point, mid_point + glm::vec3(0.0f, 100.0f, 0.0f), 1.5f))
		{
			grid[i] = -1;
			
			//SceneNode* scene_node = new SceneNode(scene_manager, parent, glm::translate(glm::mat4(1.0f), mid_point));
			//SceneLeafModel* scene_model = new SceneLeafModel(*scene_node, NULL, *cube, *terrain_material_, BasicShadowShader::getShader(), false, false);
		}
		else
		{
			//SceneNode* scene_node = new SceneNode(scene_manager, parent, glm::translate(glm::mat4(1.0f), mid_point + glm::vec3(0, 4, 0)));
			//SceneLeafModel* scene_model = new SceneLeafModel(*scene_node, NULL, *cube, *terrain_material_, BasicShadowShader::getShader(), false, false);
		}
	}
	
	std::cout << "Create the pipes." << std::endl;
	grid_ = new PipeNetworkGrid(grid, min_x, min_y, max_x, max_y, grid_size);
	//pipe_shape_ = new Shape();
	
	//findPath(glm::vec3(10, 0, 0), glm::vec3(44, 0, 25), PipeNetworkNode::N);
	for (unsigned int i = 0; i < mission_sites.size() - 1; i++)
	{
		for (unsigned int j = i + 1; j < mission_sites.size(); ++j)
		{
			for (std::vector<Structure*>::const_iterator ci = mission_sites[i]->getStructures().begin(); ci != mission_sites[i]->getStructures().end(); ++ci)
			{
				Structure* structure1 = *ci;
				for (std::vector<InspectionPoint*>::const_iterator ci = structure1->getInspectionPoints().begin(); ci != structure1->getInspectionPoints().end(); ++ci)
				{
					InspectionPoint* ip1 = *ci;
					
					for (std::vector<Structure*>::const_iterator ci = mission_sites[j]->getStructures().begin(); ci != mission_sites[j]->getStructures().end(); ++ci)
					{
						Structure* structure2 = *ci;
						for (std::vector<InspectionPoint*>::const_iterator ci2 = structure2->getInspectionPoints().begin(); ci2 != structure2->getInspectionPoints().end(); ++ci2)
						{
							InspectionPoint* ip2 = *ci2;
							if (rand() < RAND_MAX / 16) continue;
							
							findPath(ip1->getVisiblePoint(), ip2->getVisiblePoint(), PipeNetworkNode::E);
						}
					}
				}
			}
		}
	}
}

void PipeNetwork::findPath(const glm::vec3& from, const glm::vec3& to, PipeNetworkNode::PIPE_NETWORK_DIRECTION direction)
{
	int from_grid_id = grid_->getGridId(from.x, from.z);
	int to_grid_id = grid_->getGridId(to.x, to.z);
	
	std::cout << "From grid: " << from_grid_id << "(" << (from_grid_id == -1 ? -1 : grid_->grid_[from_grid_id]) << "); To grid: " << to_grid_id << "(" << (to_grid_id == -1 ? -1 : grid_->grid_[to_grid_id]) << ")." << std::endl;
	
	if (from_grid_id == -1 || grid_->grid_[from_grid_id] == -1 ||
	    to_grid_id == -1 || grid_->grid_[to_grid_id] == -1)
	{
		std::cout << "Grids are occupied..." << std::endl;
		return;
	}
	
	std::vector<PipeNetworkNode*> all_nodes;
	
	// Find a path!
	std::priority_queue<PipeNetworkNode*, std::vector<PipeNetworkNode*>, PipeNetworkNode> queue;
	
	std::vector<PipeNetworkNode*> dummy_path;
	PipeNetworkNode* start_node = new PipeNetworkNode(*grid_, NULL, from_grid_id, to_grid_id, dummy_path, direction);
	all_nodes.push_back(start_node);
	
	std::vector<PipeNetworkNode*> start_nodes;
	start_node->getAllChildren(start_nodes);
	for (std::vector<PipeNetworkNode*>::const_iterator ci = start_nodes.begin(); ci != start_nodes.end(); ++ci)
	{
		queue.push(*ci);
		all_nodes.push_back(*ci);
	}
	//queue.push(start_node);
	
	std::vector<int> closed_list(grid_->grid_);
	int ignored_children = 0;
	int checked_nodes = 0;
	while (queue.size() > 0)
	{
		PipeNetworkNode* current_node = queue.top();
		queue.pop();
		
		if (closed_list[current_node->grid_id_] == -1)
		{
			++ignored_children;
			continue;
		}
		++checked_nodes;
		
		if (checked_nodes > 100000) break;
		//std::cout << "Process: " << *current_node << "." << std::endl;
		
		closed_list[current_node->grid_id_] = -1;
		
		// Check if this is the goal.
		if (current_node->grid_id_ == to_grid_id)
		{
			const std::vector<PipeNetworkNode*>& path = current_node->path_;
			//Line* line = new Line();
			PipeNetworkNode* previous_node = NULL;
			std::vector<glm::vec3> path_vertexes;
			std::vector<glm::vec2> texture_uvs;
			std::vector<GLuint> indices;
			std::vector<glm::vec3> normals;
			
			float pipe_size = 0.25f;
			glm::vec2 current_direction(0, 0);
			
			unsigned int vertex_offset = 0;
			for (std::vector<PipeNetworkNode*>::const_iterator ci = path.begin(); ci != path.end(); ++ci)
			{
				PipeNetworkNode* node = *ci;
				std::cout << "Process: " << *node << std::endl;
				if (grid_->grid_[node->grid_id_] == -1)
				{
					grid_->grid_[node->grid_id_] == 1;
				}
				else
				{
					grid_->grid_[node->grid_id_] = grid_->grid_[node->grid_id_] + 1;
				}
				glm::vec2 location = grid_->getCoordinates(node->grid_id_);
				
				if (previous_node != NULL)
				{
					glm::vec2 prev_location = grid_->getCoordinates(previous_node->grid_id_);
					
					glm::vec2 direction = location - grid_->getCoordinates((*(ci - 1))->grid_id_);
					glm::vec2 next_location = grid_->getCoordinates((*(ci - 1))->grid_id_);
					
					glm::vec3 orthogonal(-current_direction.y, 0.0f, current_direction.x);
					orthogonal = glm::normalize(orthogonal) * pipe_size;
					
					if (current_direction == glm::vec2(0, 0))
					{
						current_direction = glm::normalize(direction);
						continue;
					}
					// Check if the pipe's direction has changed, if so we will create the pipes.
					else if (glm::dot(current_direction, glm::normalize(direction)) < 0.9f)
					{
						std::cout << "Pipe changed direction! (" << current_direction.x << ", " << current_direction.y << ") v.s. (" << direction.x << ", " << direction.y << ")" << std::endl;
						current_direction = glm::normalize(direction);
					}
					else if (ci + 1 == path.end())
					{
						std::cout << "Found the last bit of pipe!" << std::endl;
						next_location = location;
					}
					else
					{
						continue;
					}
					
					//glm::vec3 orthogonal_sqrt = glm::vec3(sqrt(orthogonal.x + orthogonal.x), 0.0f, sqrt(orthogonal.y + orthogonal.y));
					glm::vec3 orthogonal_sqrt = orthogonal * 0.7f;
					
					std::cout << "Location: (" << next_location.x << ", " << next_location.y << ")" << std::endl;
					std::cout << "Previous location: (" << prev_location.x << ", " << prev_location.y << ")" << std::endl;
					std::cout << "Orthogonal: (" << orthogonal.x << ", " << orthogonal.y << ", " << orthogonal.z << ")" << std::endl;
					
					// Only include the previous nodes for the first section.
					if (indices.empty())
					{
						path_vertexes.push_back(glm::vec3(prev_location.x, pipe_size, prev_location.y));
						path_vertexes.push_back(glm::vec3(prev_location.x, 0.7f * pipe_size, prev_location.y) + orthogonal_sqrt);
						path_vertexes.push_back(glm::vec3(prev_location.x, 0.0f, prev_location.y) + orthogonal);
						path_vertexes.push_back(glm::vec3(prev_location.x, -0.7f * pipe_size, prev_location.y) + orthogonal_sqrt);
						path_vertexes.push_back(glm::vec3(prev_location.x, -pipe_size, prev_location.y));
						path_vertexes.push_back(glm::vec3(prev_location.x, -0.7f * pipe_size, prev_location.y) - orthogonal_sqrt);
						path_vertexes.push_back(glm::vec3(prev_location.x, 0.0f, prev_location.y) - orthogonal);
						path_vertexes.push_back(glm::vec3(prev_location.x, 0.7f * pipe_size, prev_location.y) - orthogonal_sqrt);
						path_vertexes.push_back(glm::vec3(prev_location.x, 1.0f * pipe_size, prev_location.y));
						
						normals.push_back(glm::vec3(0, 1, 0));
						normals.push_back(glm::vec3(orthogonal_sqrt.x, 0.7f, orthogonal_sqrt.z));
						normals.push_back(glm::vec3(orthogonal.x, 0.0f, orthogonal.y));
						normals.push_back(glm::vec3(orthogonal_sqrt.x, -0.7f, orthogonal_sqrt.z));
						normals.push_back(glm::vec3(0, -1, 0));
						normals.push_back(glm::vec3(-orthogonal_sqrt.x, -0.7f, -orthogonal_sqrt.z));
						normals.push_back(glm::vec3(-orthogonal.x, 0.0f, -orthogonal.y));
						normals.push_back(glm::vec3(-orthogonal_sqrt.x, 0.7f, -orthogonal_sqrt.z));
						normals.push_back(glm::vec3(0, 1, 0));
						
						for (float f = 0; f <= 1.0f; f+= 0.125f)
						{
							texture_uvs.push_back(glm::vec2(0, f));
						}
					}
					
					// Render as triangles.
					path_vertexes.push_back(glm::vec3(next_location.x, pipe_size, next_location.y));
					normals.push_back(glm::vec3(0, 1, 0));
					
					path_vertexes.push_back(glm::vec3(next_location.x, 0.7f * pipe_size, next_location.y) + orthogonal_sqrt);
					normals.push_back(glm::vec3(orthogonal_sqrt.x, 0.7f, orthogonal_sqrt.z));
				
					path_vertexes.push_back(glm::vec3(next_location.x, 0.0f, next_location.y) + orthogonal);
					normals.push_back(glm::vec3(orthogonal.x, 0.0f, orthogonal.y));
					
					path_vertexes.push_back(glm::vec3(next_location.x, -0.7f * pipe_size, next_location.y) + orthogonal_sqrt);
					normals.push_back(glm::vec3(orthogonal_sqrt.x, -0.7f, orthogonal_sqrt.z));
					
					path_vertexes.push_back(glm::vec3(next_location.x, -pipe_size, next_location.y));
					normals.push_back(glm::vec3(0, -1, 0));
					
					path_vertexes.push_back(glm::vec3(next_location.x, -0.7f * pipe_size, next_location.y) - orthogonal_sqrt);
					normals.push_back(glm::vec3(-orthogonal_sqrt.x, -0.7f, -orthogonal_sqrt.z));
					
					path_vertexes.push_back(glm::vec3(next_location.x, 0.0f, next_location.y) - orthogonal);
					normals.push_back(glm::vec3(-orthogonal.x, 0.0f, -orthogonal.y));
					
					path_vertexes.push_back(glm::vec3(next_location.x, 0.7f * pipe_size, next_location.y) - orthogonal_sqrt);
					normals.push_back(glm::vec3(-orthogonal_sqrt.x, 0.7f, -orthogonal_sqrt.z));
					
					path_vertexes.push_back(glm::vec3(next_location.x, 1.0f * pipe_size, next_location.y));
					normals.push_back(glm::vec3(0, 1, 0));
					
					for (float f = 0; f <= 1.0f; f+= 0.125f)
					{
						//texture_uvs.push_back(glm::vec2(0, f));
						texture_uvs.push_back(glm::vec2(glm::distance(prev_location, next_location), f));
					}
					//18
					for (unsigned int i = 0; i < 8; ++i)
					{
						indices.push_back(vertex_offset + 0 + i);
						indices.push_back(vertex_offset + 1 + i);
						indices.push_back(vertex_offset + 9 + i);
						
						indices.push_back(vertex_offset + 9 + i);
						indices.push_back(vertex_offset + 1 + i);
						indices.push_back(vertex_offset + 10 + i);
						
						std::cout << (vertex_offset + 0 + i * 2) << ", " << (vertex_offset + 1 + i * 2) << ", " << (vertex_offset + 2 + i * 2) << " - " << (vertex_offset + 2 + i * 2) <<  ", " << (vertex_offset + 1 + i * 2) << ", " << (vertex_offset + 3 + i * 2) << std::endl;
					}
					vertex_offset += 9;
					previous_node = *(ci - 1);
				}
				
				if (previous_node == NULL)
				{
					previous_node = node;
				}
			}
			std::cout << "Vertices: " << path_vertexes.size() << "; Tex UVs: " << texture_uvs.size() << "; Indices: " << indices.size() << "; Normals: " << normals.size() << "." << std::endl;
			for (std::vector<glm::vec3>::const_iterator ci = path_vertexes.begin(); ci != path_vertexes.end(); ++ci)
			{
				std::cout << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ")" << std::endl;
			}
			//line->setVertexBuffer(path_vertexes);
			Shape* shape = new Shape(path_vertexes, texture_uvs, indices, normals);
			
			SceneLeafModel* scene_model = new SceneLeafModel(*this, NULL, *shape, *pipe_material_, BasicShadowShader::getShader(), false, false, OBJECT, ShadowRenderer::NO_SHADOW);
			
			std::cout << "Found a solution! " << path_vertexes.size() << "; Checked nodes: " << checked_nodes << "; skipped: " << ignored_children << std::endl;
			break;
		}
		
		std::vector<PipeNetworkNode*> children;
		current_node->getChildren(children);
		//std::cout << "Create children: " << std::endl;
		for (std::vector<PipeNetworkNode*>::const_iterator ci = children.begin(); ci != children.end(); ++ci)
		{
			//std::cout << "\t* " << **ci<< "." << std::endl;
			queue.push(*ci);
			
			all_nodes.push_back(*ci);
		}
	}
	
	for (std::vector<PipeNetworkNode*>::const_iterator ci = all_nodes.begin(); ci != all_nodes.end(); ++ci)
	{
		delete *ci;
	}
	
	std::cout << "No solution found!" << std::endl;
	std::cout << "Checked nodes: " << checked_nodes << "; skipped: " << ignored_children << std::endl;
}

std::ostream& operator<<(std::ostream& os, const PipeNetworkNode& node)
{
	const glm::vec2& location = node.grid_->getCoordinates(node.grid_id_);
	os << "[PipeNetworkNode] Grid id: " << node.grid_id_ << " (" << location.x << ", " << location.y << "). Cost: " << node.cost_ << " + " << node.estimated_cost_to_goal_ << "; Achieved by: " << node.direction_ << ".";
	return os;
}
