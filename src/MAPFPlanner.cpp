#include <MAPFPlanner.h>
#include <random>
#include <unordered_set>


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
    cout << "planner initialize done" << endl;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    std::vector<std::list<std::pair<int, int>>> paths;
    for (int i = 0; i < env->num_of_agents; i++) {
        paths.push_back(single_agent_plan(env->curr_states[i].location,
                                            env->curr_states[i].orientation,
                                            env->goal_locations[i].empty() ? env->curr_states[i].location : env->goal_locations[i].front().first));
    }
    // std::cout << "Paths Found!" << std::endl;
    // Detect conflicts at the current timestep
    // std::unordered_set<conflict, conflictHash, conflictEqual> conflicts = detectConflicts(paths,timestep);

    // Resolve conflicts using CBS (or other conflict resolution method)
    // resolveConflicts(paths, conflicts);
    // std::cout << "Conflicts Resolved!" << std::endl;

    // Go through the paths and find the conflicts and resolve them

    // Resolve conflicts using CBS (or other conflict resolution method)
    // std::cout << "Conflicts Resolved!" << std::endl;

    // Update the actions based on the resolved paths
    for (int i = 0; i < env->num_of_agents; i++) {
        if (!paths[i].empty()) {
            std::pair<int, int> next_position = paths[i].front();

            if (next_position.first != env->curr_states[i].location) {
                actions[i] = Action::FW;
            } else if (next_position.second != env->curr_states[i].orientation) {
                int incr = next_position.second - env->curr_states[i].orientation;
                if (incr == 1 || incr == -3) {
                    actions[i] = Action::CR;
                } else if (incr == -1 || incr == 3) {
                    actions[i] = Action::CCR;
                }
            }
        }
    }

    // actions = std::vector<Action>(env->curr_states.size(), Action::W);
    // for (int i = 0; i < env->num_of_agents; i++) 
    // {
    //     list<pair<int,int>> path;
    //     if (env->goal_locations[i].empty()) 
    //     {
    //         path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
    //     } 
    //     else 
    //     {
    //         path = single_agent_plan(env->curr_states[i].location,
    //                                 env->curr_states[i].orientation,
    //                                 env->goal_locations[i].front().first);
    //     }
    //     if (path.front().first != env->curr_states[i].location)
    //     {
    //         actions[i] = Action::FW; //forward action
    //     } 
    //     else if (path.front().second!= env->curr_states[i].orientation)
    //     {
    //         int incr = path.front().second - env->curr_states[i].orientation;
    //         if (incr == 1 || incr == -3)
    //         {
    //             actions[i] = Action::CR; //C--counter clockwise rotate
    //         } 
    //         else if (incr == -1 || incr == 3)
    //         {
    //             actions[i] = Action::CCR; //CCR--clockwise rotate
    //         } 
    //     }
    
    // }


    // Implement the getMAPF function here
    // vector<vector<pair<int,int>>> paths;
    // getMAPFPlan(env->curr_states, paths, time_limit);
    // for (int i = 0; i < env->num_of_agents; i++) 
    // {
    //     if (!paths[i].empty()) 
    //     {
    //         std::pair<int, int> next_position = paths[i].front();

    //         if (next_position.first != env->curr_states[i].location) 
    //         {
    //             actions[i] = Action::FW;
    //         } 
    //         else if (next_position.second != env->curr_states[i].orientation) 
    //         {
    //             int incr = next_position.second - env->curr_states[i].orientation;
    //             if (incr == 1 || incr == -3) 
    //             {
    //                 actions[i] = Action::CR;
    //             } 
    //             else if (incr == -1 || incr == 3) 
    //             {
    //                 actions[i] = Action::CCR;
    //             }
    //         }
    //     }
    // }
    
    
  return;
}


list<pair<int,int>> MAPFPlanner::single_agent_plan(int start,int start_direct,int end)
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
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
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

