#include <MAPFPlanner.h>
#include <random>

/**
 * @fn CT_node constructor
 * @brief Construct a new CT_node::CT_node object
*/
CT_node::CT_node(/* args */)
{
}

CT_node::~CT_node()
{
}


/**
 * @fn CBS functions
 * 
*/
void MAPFPlanner::naive_CBS()
{
    constraint_format c1(0, 0, 1, 0);
    // create a root node with empty constraint variable
    std::shared_ptr<CT_node> root_node = std::make_shared<CT_node>();

    // calculate paths for all agents using A*(heuristics are already calculated in initialize function)
    // store it the root node
    list<list<pair<int,int>>> solution;
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else 
        {
            path = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first, {c1});
        }
        solution.push_back(path);
    }
    root_node->solution = solution;
    
    
    // calculate sum-of-cost and store it in root node

    // while loop till open_list is empty
        // pop the best CT_node with lowest sum-of-cost (create a comparator function to compare CT_nodes)
        
        // check for conflicts in the paths (create a conflict finding function)

        // if no conflict, return the solution as this is the goal

        // for each conflict
            // create new CT_nodes 

            // add the constraint to the new CT_nodes, which is Parent CT_node's constraint + conflict

            // calculate path for only this agent
            // copy solution from parent CT_node to new CT_node
            // update the solution in new CT_node
            
            // calculate sum-of-cost and store it in new CT_node
            // if cost is less than infinity
                // push the new CT_nodes in open_list


}


struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};



void MAPFPlanner::initialize(int preprocess_time_limit)
{
    cout << "****************************************************" << endl;
    cout << "planner preprocess time limit: " << preprocess_time_limit << endl;

    // calculate heuristic for all agents and store
    cout << "planner initialize done" << endl;
}

int MAPFPlanner::sum_of_costs(vector<list<pair<int,int>>> paths){
    int sum = 0;
    for(auto path: paths){
        sum += path.size();
    }
    return sum;

}

// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{   
    constraint_format c1(0, 0, 1, 0);
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else 
        {
            path = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first, {c1});
        }
        if (path.front().first != env->curr_states[i].location)
        {
            actions[i] = Action::FW; //forward action
        } 
        else if (path.front().second!= env->curr_states[i].orientation)
        {
            int incr = path.front().second - env->curr_states[i].orientation;
            if (incr == 1 || incr == -3)
            {
                actions[i] = Action::CR; //C--counter clockwise rotate
            } 
            else if (incr == -1 || incr == 3)
            {
                actions[i] = Action::CCR; //CCR--clockwise rotate
            } 
        }
    
    }

    
  return;
}

bool MAPFPlanner::found_node(vector<constraint_format> constraints, AstarNode* node){
    for(auto n: constraints){
        if(n.vertex_1 ==node->location && n.t==node->t){
            return true;
        }
    }
    return false;
}

list<pair<int,int>> MAPFPlanner::single_agent_plan(int start,int start_direct,int end, vector<constraint_format> constraints)
{
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end)
        {
            while(curr->parent!=NULL) 
            {
                path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            break;
        }
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
        for (const pair<int,int>& neighbor: neighbors)
        {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {   
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
        
                {   AstarNode node(old->location,old->direction,curr->g+1,old->h+curr->g+1,curr->t+1,curr);
                    if(!found_node(constraints, &node)){ 
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                    old->t=curr->t+1;
                    }
                }
            }
            else
            {   
                
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end),curr->t ,curr);
                if(!found_node(constraints, next_node)){
                    open_list.push(next_node);
                }
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return path;
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool MAPFPlanner::validateMove(int loc, int loc2)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}


list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction)
{
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));
    neighbors.emplace_back(make_pair(location,direction)); //wait
    return neighbors;
}

// Resolve conflicts using CBS (or other conflict resolution method)

int MAPFPlanner::Node_cost(std::vector<std::list<std::pair<int, int>>>& paths)
{
    // Return the cost which is the sum of the length of all the paths without using a loop
    int cost = 0;
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        cost += paths[i].size();
    }
    return cost;
}

void MAPFPlanner::combinationsUtil(const std::vector<int>& arr, std::vector<std::vector<int>>& result, std::vector<int>& combination, int start, int end, int index, int k) 
{
    if (index == k) {
        result.push_back(combination);
        return;
    }

    for (int i = start; i <= end && end - i + 1 >= k - index; i++) {
        combination[index] = arr[i];
        combinationsUtil(arr, result, combination, i + 1, end, index + 1, k);
    }
}


std::vector<std::vector<int>> MAPFPlanner::combinations(const std::vector<int>& arr, int k) {
    std::vector<std::vector<int>> result;
    std::vector<int> combination(k);
    combinationsUtil(arr, result, combination, 0, arr.size() - 1, 0, k);
    return result;
}

conflict MAPFPlanner::findVertexConflicts(const std::vector<std::list<std::pair<int, int>>>& paths) 
{
    std::vector<int> agentIndices(paths.size());
    std::iota(agentIndices.begin(), agentIndices.end(), 0);  // Fill with 0, 1, ..., n-1

    std::vector<std::vector<int>> agentCombinations = combinations(agentIndices, 2);

    // Check conflicts for each combination of 2 agents
    for (const auto& combination : agentCombinations) {
        // In the combination, combination[0] is the first agent and combination[1] is the second agent
        int agent1 = combination[0];
        int agent2 = combination[1];

        // Find the agent with the shorter path
        int shorterAgent = (paths[agent1].size() < paths[agent2].size()) ? agent1 : agent2;
        int longerAgent = (shorterAgent == agent1) ? agent2 : agent1;

        // Run a loop of length equal to the length of the shorter path where ith element of both paths are checked if they are equal
        for (int i = 0; i < paths[shorterAgent].size(); i++) 
        {
            // Create variable which hold the value of ith path element of both agents without using auto
            std::pair<int, int> shorterAgentPathElement = *std::next(paths[shorterAgent].begin(), i);
            // Check if the two agents are at the same location at the same timestep
            if (*std::next(paths[shorterAgent].begin(), i) == *std::next(paths[longerAgent].begin(), i)) 
            {
                return conflict(shorterAgent, longerAgent, shorterAgentPathElement, i);
            }
        }

    }
    return conflict();
}

//create a function to get the first edge conflicts
conflict MAPFPlanner::findEdgeConflicts(const std::vector<std::list<std::pair<int, int>>>& paths) 
{
    std::vector<int> agentIndices(paths.size());
    std::vector<std::vector<int>> agentCombinations = combinations(agentIndices, 2);

    // Check conflicts for each combination of 2 agents
    for (const auto& combination : agentCombinations) {
        // In the combination, combination[0] is the first agent and combination[1] is the second agent
        int agent1 = combination[0];
        int agent2 = combination[1];

        // Find the agent with the shorter path
        int shorterAgent = (paths[agent1].size() < paths[agent2].size()) ? agent1 : agent2;
        int longerAgent = (shorterAgent == agent1) ? agent2 : agent1;

        // Run a loop of length equal to the length of the shorter path where ith element of both paths are checked if they are equal
        for (int i = 0; i < paths[shorterAgent].size()-1; i++) 
        {
            // Create variable which hold the value of ith and i+1th path element of both agents without using auto
            std::pair<int, int> shorterAgentPath_Current = *std::next(paths[shorterAgent].begin(), i);
            std::pair<int, int> shorterAgentPath_Next = *std::next(paths[shorterAgent].begin(), i+1);
            std::pair<int, int> longerAgentPath_Current = *std::next(paths[longerAgent].begin(), i);
            std::pair<int, int> longerAgentPath_Next = *std::next(paths[longerAgent].begin(), i+1);
            // Check if the two agents are at the same location at the same timestep
            if (shorterAgentPath_Current == longerAgentPath_Next && shorterAgentPath_Next == longerAgentPath_Current) 
            {
                return conflict(shorterAgent, longerAgent, shorterAgentPath_Current, longerAgentPath_Current, i);
            }
        }

    }
    return conflict();
}

//Create a function to convert x and y coordinates to a single integer for a map represented as a vector]
int MAPFPlanner::convertToSingleInt(int x, int y)
{
    return x*env->cols + y;
}

// Create a function to convert a single integer to x and y coordinates for a map represented as a vector
std::pair<int, int> MAPFPlanner::convertToPair(int singleInt)
{
    int x = singleInt/env->cols;
    int y = singleInt%env->cols;
    return std::make_pair(x, y);
}

// Function which takes in the an edge conflict and returns a constraint of the same
constraint_format MAPFPlanner::convertToConstraint(conflict Conflict) 
{
    if (Conflict.vertex2.first == -1 && Conflict.vertex2.second == -1) // Check if the conflict is a vertex conflict
    {
        return constraint_format(Conflict.agent1, convertToSingleInt(Conflict.vertex1.first, Conflict.vertex1.second), Conflict.timestep);
    } 
    else // If the conflict is an edge conflict
    {
        return constraint_format(Conflict.agent1, convertToSingleInt(Conflict.vertex1.first, Conflict.vertex1.second), convertToSingleInt(Conflict.vertex2.first, Conflict.vertex2.second), Conflict.timestep);
    }
}

