#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include <limits>




// Define a structure to represent a conflict
struct conflict {
    int agent1;
    int agent2;
    int vertex1;
    int vertex2;
    int timestep;

    // Default constructor
    conflict() : agent1(-1), agent2(-1), vertex1(-1), vertex2(-1), timestep(-1) {}
    
    // Vertex Conflict constructor
    conflict(int a1, int a2, int _vertex1, int t) : agent1(a1), agent2(a2), vertex1(_vertex1), timestep(t) {
        vertex2 = -1;
    }

    // Edge Conflict constructor
    conflict(int a1, int a2, int _vertex1, int _vertex2, int t) : agent1(a1), agent2(a2), vertex1(_vertex1), vertex2(_vertex2), timestep(t) {}
};

struct conflictEqual {
    bool operator()(const conflict& a, const conflict& b) const {
        return a.agent1 == b.agent1 && a.agent2 == b.agent2 && a.timestep == b.timestep;
    }
};

struct constraint_format{
    int agent_id,t, vertex_1, vertex_2;
    // Edge Constraint Constructor
    constraint_format(int _agent_id, int _vertex_1, int _vertex_2, int t): agent_id(_agent_id), vertex_1(_vertex_1), vertex_2(_vertex_2), t(t) {}
    // Vertex Constraint Constructor
    constraint_format(int _agent_id, int _vertex_1, int t): agent_id(_agent_id), vertex_1(_vertex_1), t(t) {
        vertex_2 = -1;
    }
};

struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};

class MAPFPlanner
{
public:
    SharedEnvironment* env;
    std::vector<std::list<std::pair<int,int>>> CBS_solution; 
    std::vector<std::list<std::pair<int,int>>> carry_forward_soln; 
    unordered_map<int, unordered_map<int,int>> heuristics_map; // start location is key
	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // CBS implementation function
    void naive_CBS(vector<bool> find_new_path);

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan(int agent_id, int start,int start_direct, int end, vector<constraint_format> constraints);
    int getManhattanDistance(int loc1, int loc2);
    int getEuclideanDistance(int loc1, int loc2);
    int sum_of_costs(vector<list<pair<int,int>>> paths);
    bool found_node(int agent_id, vector<constraint_format> constraints, AstarNode* node);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);

    //My work
    bool validateCell(int loc);
    void combinationsUtil(const std::vector<int>& arr, std::vector<std::vector<int>>& result, std::vector<int>& combination, int start, int end, int index, int k); 
    std::vector<std::vector<int>> combinations(std::vector<int> arr);
    conflict findConflicts(const std::vector<std::list<std::pair<int, int>>>& paths); 
    // conflict findEdgeConflicts(const std::vector<std::list<std::pair<int, int>>>& paths);
    int convertToSingleInt(int x, int y);
    std::pair<int, int> convertToPair(int singleInt);
    vector<constraint_format> convertToConstraint(conflict edgeConflict);

    void computeHeuristic();
    int get_heuristic(int start_loc, int start_dir, int goal_loc);
};

/**
 * @class CT_node
 * @brief Constraint Tree Node
 * 
*/
class CT_node
{
private:
    /* data */
public:

    // store constraint : vector of tuple<agent_id, map_index, time_step> 
    vector<constraint_format> node_constraints;
    // solution basically list of list pair<int,int> where pair is index and direction
    vector<list<pair<int,int>>> node_solution;
    // sum-of-cost // float or int
    int SOC;

    // parent pointer
    std::shared_ptr<CT_node> parent_ptr;
    // right pointer 
    std::shared_ptr<CT_node> right_ptr;
    // left pointer
    std::shared_ptr<CT_node> left_ptr;


    CT_node(/* args */);
    ~CT_node();
};