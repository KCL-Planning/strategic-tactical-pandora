#ifndef DEMO_PANDORA_PROBLEM_GENERATOR_H
#define DEMO_PANDORA_PROBLEM_GENERATOR_H

class SceneManager;
class SceneNode;

class ProblemGenerator
{
public:
	ProblemGenerator(SceneManager& scene_manager, SceneNode& terrain_node_);
	
	
	void addValve();
	void addChain();
	void addInspectionMission();
	
	void generateProblem(unsigned int nr_valves, unsigned int nr_chains, unsigned int nr_inspection_missions);
private:
	
	void generateMissionSite(unsigned int nr_valves, unsigned int nr_chains, unsigned int nr_inspection_missions);
	
	SceneManager* scene_manager_;
	SceneNode* terrain_node_;
};

#endif
