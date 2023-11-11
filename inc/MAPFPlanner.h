#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end, vector<constraint> constraints);
    int getManhattanDistance(int loc1, int loc2);
    int sum_of_costs(vector<list<pair<int,int>>> paths);
    bool found_node(vector<constraint> constraints, AstarNode* node);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);
};
