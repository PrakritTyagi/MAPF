#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include <unordered_set>

// Define a structure to represent a conflict
struct conflict {
    int agent1;
    int agent2;
    std::pair<int, int> vertex1;
    std::pair<int, int> vertex2;
    int timestep;

    // Default constructor
    conflict() : agent1(-1), agent2(-1), vertex1(std::make_pair(-1, -1)), vertex2(std::make_pair(-1, -1)), timestep(-1) {}
    
    // Vertex Conflict constructor
    conflict(int a1, int a2, std::pair<int, int> _vertex1, int t) : agent1(a1), agent2(a2), vertex1(_vertex1), timestep(t) {
        vertex2 = std::make_pair(-1, -1);
    }

    // Edge Conflict constructor
    conflict(int a1, int a2, std::pair<int, int> _vertex1, std::pair<int, int> _vertex2, int t) : agent1(a1), agent2(a2), vertex1(_vertex1), vertex2(_vertex2), timestep(t) {}
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
    int Node_cost(std::vector<std::list<std::pair<int, int>>>& paths);

    void combinationsUtil(const std::vector<int>& arr, std::vector<std::vector<int>>& result, std::vector<int>& combination, int start, int end, int index, int k); 
    std::vector<std::vector<int>> combinations(const std::vector<int>& arr, int k);
    conflict findVertexConflicts(const std::vector<std::list<std::pair<int, int>>>& paths); 
    conflict findEdgeConflicts(const std::vector<std::list<std::pair<int, int>>>& paths);
};
