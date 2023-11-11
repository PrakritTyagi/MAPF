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
    std::unordered_set<conflict, conflictHash, conflictEqual> conflicts;
    int timestep = 0; // define timestep
    for (int i = 0; i < env->num_of_agents; i++) {
        for (int j = i + 1; j < env->num_of_agents; j++) {
            // Check if agents i and j collide at the current timestep
            auto it_i = std::next(paths[i].begin(), timestep);
            auto it_j = std::next(paths[j].begin(), timestep);
            if (it_i != paths[i].end() && it_j != paths[j].end() && *it_i == *it_j) {
                conflicts.emplace(i, j, timestep);
            }
            timestep++;
        }
        timestep = 0;
    }

    // Resolve conflicts using CBS (or other conflict resolution method)
    resolveConflicts(paths, conflicts);
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

// Implement a function that uses the conflict based search to find a solution to CBS
void MAPFPlanner::getMAPFPlan(vector<State> & curr_states, vector<vector<pair<int,int>>> & paths, int time_limit)
{
    // Write a conflict based search algorithm here
    std::vector<Action> actions(env->curr_states.size(), Action::W);

    // Continue planning until the time limit is reached
    for (int timestep = 0; timestep < time_limit; timestep++) {
        // Compute paths for each agent at the current timestep
        std::vector<std::list<std::pair<int, int>>> paths;
        for (int i = 0; i < env->num_of_agents; i++) {
            paths.push_back(single_agent_plan(env->curr_states[i].location,
                                              env->curr_states[i].orientation,
                                              env->goal_locations[i].empty() ? env->curr_states[i].location : env->goal_locations[i].front().first));
        }
        // std::cout << "Paths Found!" << std::endl;
        // Detect conflicts at the current timestep
        std::unordered_set<conflict, conflictHash, conflictEqual> conflicts = detectConflicts(paths, timestep);

        // Resolve conflicts using CBS (or other conflict resolution method)
        resolveConflicts(paths, conflicts);
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

        // Update the current states for the next timestep
        for (int i = 0; i < env->num_of_agents; i++) {
            if (!paths[i].empty()) {
                env->curr_states[i].location = paths[i].front().first;
                env->curr_states[i].orientation = paths[i].front().second;
                paths[i].pop_front();
            }
        }
    }

}

// Helper function to detect conflicts at a given timestep
std::unordered_set<conflict, conflictHash, conflictEqual> MAPFPlanner::detectConflicts(
        const std::vector<std::list<std::pair<int, int>>>& paths, int timestep) 
        {
        std::unordered_set<conflict, conflictHash, conflictEqual> conflicts;

        for (int i = 0; i < env->num_of_agents; i++) {
            for (int j = i + 1; j < env->num_of_agents; j++) {
                // Check if agents i and j collide at the current timestep
                auto it_i = std::next(paths[i].begin(), timestep);
                auto it_j = std::next(paths[j].begin(), timestep);
                if (it_i != paths[i].end() && it_j != paths[j].end() && *it_i == *it_j) {
                    conflicts.emplace(i, j, timestep);
                }
            }
        }

        return conflicts;
    }

// Resolve conflicts using CBS (or other conflict resolution method)
void MAPFPlanner::resolveConflicts(std::vector<std::list<std::pair<int, int>>>& paths,
                                   const std::unordered_set<conflict, conflictHash, conflictEqual>& conflicts) 
{
    for (const auto& conflict : conflicts) 
    {
        // In this simple example, resolve conflicts by giving priority to one of the agents
        int agentToMove = (rand() % 2 == 0) ? conflict.agent1 : conflict.agent2;

        // Modify the path of the agent to avoid the conflict
        if (std::distance(paths[agentToMove].begin(), paths[agentToMove].end()) > conflict.timestep + 1) {
            // Move the agent to a neighboring cell (you might want to implement a more sophisticated strategy)
            auto it = paths[agentToMove].begin();
            std::advance(it, conflict.timestep + 1);
            *it = std::make_pair(it->first + 1, it->second); // move the agent to a neighboring cell
        }
    }
}

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


    for (int i = start; i <= end && end - i + 1 >= k - index; i++) {
        combination[index] = arr[i];
        combinationsUtil(arr, result, combination, i + 1, end, index + 1, k);
    }
}

std::vector<std::vector<int>> combinations(const std::vector<int>& arr, int k) {
    std::vector<std::vector<int>> result;
    std::vector<int> combination(k);
    combinationsUtil(arr, result, combination, 0, arr.size() - 1, 0, k);
    return result;
}
