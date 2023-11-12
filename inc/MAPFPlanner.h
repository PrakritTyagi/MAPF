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

    // CBS implementation function
    virtual void naive_CBS();

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);
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
    vector<tuple<int,int,int>> constraints;
    // solution basically list of list pair<int,int> where pair is index and direction
    list<list<pair<int,int>>> solution;
    // sum-of-cost // float or int
    int sum_of_cost;

    // parent pointer
    std::shared_ptr<CT_node> parent_ptr;
    // right pointer 
    std::shared_ptr<CT_node> right_ptr;
    // left pointer
    std::shared_ptr<CT_node> left_ptr;


    CT_node(/* args */);
    ~CT_node();
};

