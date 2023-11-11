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
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end, vector<constraint_format> constraints);
    int getManhattanDistance(int loc1, int loc2);
    int sum_of_costs(vector<list<pair<int,int>>> paths);
    bool found_node(vector<constraint_format> constraints, AstarNode* node);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);
};

struct constraint_format{
    int agent_id,t, vertex_1, vertex_2;
    constraint_format(int _agent_id, int _vertex_1, int _vertex_2, int t): agent_id(_agent_id), vertex_1(_vertex_1), vertex_2(_vertex_2), t(t) {}
    constraint_format(int _agent_id, int _vertex_1, int t): agent_id(_agent_id), vertex_1(_vertex_1), t(t) {}
};
