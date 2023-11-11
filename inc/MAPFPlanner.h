#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include <unordered_set>

// Define a structure to represent a conflict
struct conflict {
    int agent1;
    int agent2;
    int timestep;

    conflict(int a1, int a2, int t) : agent1(a1), agent2(a2), timestep(t) {}
};

// Define hash and equality functions for Conflict
struct conflictHash {
    size_t operator()(const conflict& conflict) const {
        return std::hash<int>()(conflict.agent1) ^ std::hash<int>()(conflict.agent2) ^ std::hash<int>()(conflict.timestep);
    }
};

struct conflictEqual {
    bool operator()(const conflict& a, const conflict& b) const {
        return a.agent1 == b.agent1 && a.agent2 == b.agent2 && a.timestep == b.timestep;
    }
};


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
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);

    //My work
    void getMAPFPlan(vector<State> & curr_states, vector<vector<pair<int,int>>> & paths, int time_limit);
    std::unordered_set<conflict, conflictHash, conflictEqual> detectConflicts(
    const std::vector<std::list<std::pair<int, int>>>& paths, int timestep);
    void resolveConflicts(std::vector<std::list<std::pair<int, int>>>& paths,
                                   const std::unordered_set<conflict, conflictHash, conflictEqual>& conflicts);
    int Node_cost(std::vector<std::list<std::pair<int, int>>>& paths);

};
