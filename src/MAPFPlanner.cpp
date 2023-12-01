#include <MAPFPlanner.h>
#include <random>

vector<list<pair<int,int>>> CBS_solution;

/**
 * @fn CT_node constructor
 * @brief Construct a new CT_node::CT_node object
*/
CT_node::CT_node(/* args */)
{
    this->parent_ptr = nullptr;
    this->right_ptr = nullptr;
    this->left_ptr = nullptr;
    this->SOC = 0;
    this->node_constraints = {};
    this->node_solution = {};
}

CT_node::~CT_node()
{
}

/**
 * @fn Contraint tree open list comparator
 * @brief Compare two CT_nodes based on their sum-of-cost
*/
struct CT_CMP
{
    bool operator()(std::shared_ptr<CT_node> a, std::shared_ptr<CT_node> b)
    {
        return a->SOC > b->SOC;
    }
};

/**
 * @fn CBS functions
 * 
*/
void MAPFPlanner::naive_CBS()
{   
    // create a root node with empty constraint variable
    std::shared_ptr<CT_node> root_node = std::make_shared<CT_node>();

    // calculate paths for all agents using A*(TODO: heuristics are already calculated in initialize function)
    // store it the root node
    vector<list<pair<int,int>>> solution;
    cout<<"Generating Initial Solution for agents"<<endl;
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
                                    env->goal_locations[i].front().first, root_node->node_constraints);
        }
        solution.push_back(path);
    }
    root_node->node_solution = solution;
    printf("Initial Solution Generated -> Size of solution vector %ld \n",root_node->node_solution.size());
    // calculate sum-of-cost and store it in root node
    root_node->SOC = sum_of_costs(root_node->node_solution);
    
    priority_queue<std::shared_ptr<CT_node>,vector<std::shared_ptr<CT_node>>,CT_CMP> OPEN_LIST;
    OPEN_LIST.push(root_node);
    // while loop till open_list is empty
    while (!OPEN_LIST.empty())
    {   
        printf("OPEN_LIST size %ld \n",OPEN_LIST.size());
        // pop the best CT_node with lowest sum-of-cost (create a comparator function to compare CT_nodes)
        std::shared_ptr<CT_node> curr_node = OPEN_LIST.top();
        OPEN_LIST.pop();

        // check for conflicts in the paths (create a conflict finding function)
        conflict CONFLICT = findVertexConflicts(curr_node->node_solution);
        // if no conflict, return the solution as this is the goal
        if (CONFLICT.agent1 == -1 && CONFLICT.agent2 == -1) 
        {
            conflict CONFLICT = findEdgeConflicts(curr_node->node_solution);
            if (CONFLICT.agent1 == -1 && CONFLICT.agent2 == -1) 
            {
                // return the solution
                cout << "Solution found" << endl;
                CBS_solution = curr_node->node_solution;
                return;
            }
        }
        // if conflict, create two new CT_nodes
        // first node
        vector<constraint_format> current_constraint_stack = convertToConstraint(CONFLICT);

        std::shared_ptr<CT_node> left_node = std::make_shared<CT_node>();
        curr_node->left_ptr = left_node;
        left_node->node_constraints = curr_node->node_constraints;
        left_node->node_constraints.push_back(current_constraint_stack[0]);
        left_node->node_solution = curr_node->node_solution;
        list<pair<int,int>> new_path = single_agent_plan(env->curr_states[CONFLICT.agent1].location,
                                    env->curr_states[CONFLICT.agent1].orientation,
                                    env->goal_locations[CONFLICT.agent1].front().first, left_node->node_constraints); // TODO: current_constraint_Stack[0].agent_id
        left_node->node_solution[CONFLICT.agent1] = new_path;
        left_node->SOC = sum_of_costs(left_node->node_solution);
        left_node->parent_ptr = curr_node;

        // second node
        std::shared_ptr<CT_node> right_node = std::make_shared<CT_node>();
        curr_node->right_ptr = right_node;
        right_node->node_constraints = curr_node->node_constraints;
        right_node->node_constraints.push_back(current_constraint_stack[1]);
        right_node->node_solution = curr_node->node_solution;
        new_path = single_agent_plan(env->curr_states[CONFLICT.agent2].location,
                                    env->curr_states[CONFLICT.agent2].orientation,
                                    env->goal_locations[CONFLICT.agent2].front().first, right_node->node_constraints);
        right_node->node_solution[CONFLICT.agent2] = new_path;
        right_node->SOC = sum_of_costs(right_node->node_solution);
        right_node->parent_ptr = curr_node;

        // push the two new CT_nodes in open_list
        if(left_node->SOC < INT64_MAX)
            OPEN_LIST.push(left_node);
        if(right_node->SOC < INT64_MAX)
            OPEN_LIST.push(right_node);

        // Empty the current node's solution and constraints to free up memory
        curr_node->node_constraints.clear();
        curr_node->node_solution.clear();
    }
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
// void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
// {   
//     // static bool run_cbs=true;
//     // if(run_cbs){
//     //     cout<<"Entering Planner"<<endl;
//     //     naive_CBS() ;
//     //     run_cbs=false;
//     //     cout<<"Exiting Planner"<<endl;
//     // }
//     // else
//     //     cout<<"Using existing CBS solution"<<endl;
//     actions = std::vector<Action>(env->curr_states.size(), Action::W);
//     for (int i = 0; i < env->num_of_agents; i++) 
//     {
//         list<pair<int,int>> path;
//         if (env->goal_locations[i].empty()) 
//         {
//             path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
//         } 
//         else 
//         {
//             path = CBS_solution[i];
//         }
//         if (path.front().first != env->curr_states[i].location)
//         {
//             actions[i] = Action::FW; //forward action
//         } 
//         else if (path.front().second!= env->curr_states[i].orientation)
//         {
//             int incr = path.front().second - env->curr_states[i].orientation;
//             if (incr == 1 || incr == -3)
//             {
//                 actions[i] = Action::CR; //C--counter clockwise rotate
//             } 
//             else if (incr == -1 || incr == 3)
//             {
//                 actions[i] = Action::CCR; //CCR--clockwise rotate
//             } 
//         }
//         CBS_solution[i].pop_front();
    
//     }

    
//   return;
// }

bool MAPFPlanner::found_node(vector<constraint_format> constraints, AstarNode* node){
    for(auto n: constraints){
        // If Vertex Constraint
        if(n.vertex_2==-1){
            if(n.vertex_1 ==node->location && n.t==node->t){
            return true;
            }
        }
        // If Edge Constraint
        else{
            if((n.vertex_2 ==node->location && n.t+1==node->t) ||(n.vertex_1 ==node->location && n.t+1==node->t))  {
            return true;
            }
        }
    }
    return false;
}

// bool MAPFPlanner::found_node_NT(vector<constraint_format> constraints, AstarNode* node){
//     for(auto n: constraints){
//         // If Vertex Constraint
//         if(n.vertex_2==-1){
//             if(n.vertex_1 ==node->location){
//             return true;
//             }
//         }
//         // If Edge Constraint
//         else{
//             if((n.vertex_2 ==node->location) ||(n.vertex_1 ==node->location))  {
//             return true;
//             }
//         }
//     }
//     return false;
// }

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
            std::pair<int, int> longerAgentPathElement = *std::next(paths[longerAgent].begin(), i);
            // Check if the two agents are at the same location at the same timestep
            if (shorterAgentPathElement.first == longerAgentPathElement.first) 
            {
                return conflict(shorterAgent, longerAgent, shorterAgentPathElement.first, i);
            }
        }

    }
    return conflict();
}

std::vector<std::pair<int, int>> MAPFPlanner::generateCombinations(int n) {
    std::vector<std::pair<int, int>> combinations;

    for (int i = 0; i < n - 1; ++i) {
        for (int j = i + 1; j < n; ++j) {
            combinations.push_back(std::make_pair(i, j));
        }
    }

    return combinations;
}


list<conflict> MAPFPlanner::findVertexConflictsList(const std::vector<std::list<std::pair<int, int>>>& paths)
{
    std::vector<std::pair<int, int>> agentCombinations = generateCombinations(paths.size());
    list<conflict> conflicts;
    // Check conflicts for each combination of 2 agents
    for (const auto& combination : agentCombinations) {
        // In the combination, combination[0] is the first agent and combination[1] is the second agent
        int agent1 = combination.first;
        int agent2 = combination.second;
        // if (agent1 == 9 && agent2 == 16){
        // printf("Agent 1 : Current : %d | Next : %d", env->curr_states[agent1].location, paths[agent1].front().first);
        // printf("Agent 2 : Current : %d | Next : %d", env->curr_states[agent2].location, paths[agent2].front().first);
        // }
        if(env->curr_states[agent1].location == paths[agent1].front().first && env->curr_states[agent1].location == paths[agent2].front().first)
        {
            conflicts.push_back(conflict(agent1, agent2, env->curr_states[agent1].location, 0));
            continue;
        }
        else if(env->curr_states[agent2].location == paths[agent2].front().first && env->curr_states[agent2].location == paths[agent1].front().first)
        {
            conflicts.push_back(conflict(agent1, agent2, env->curr_states[agent2].location, 0));
            continue;
        }
        else if(env->curr_states[agent1].location == paths[agent2].front().first && env->curr_states[agent2].location == paths[agent1].front().first)
        {
            conflicts.push_back(conflict(agent1, agent2, env->curr_states[agent1].location, 0));
            continue;
        }

        // Find the agent with the shorter path
        int shorterAgent = (paths[agent1].size() < paths[agent2].size()) ? agent1 : agent2;
        int longerAgent = (shorterAgent == agent1) ? agent2 : agent1;

        // Run a loop of length equal to the length of the shorter path where ith element of both paths are checked if they are equal
        for (int i = 0; i < paths[shorterAgent].size(); i++) 
        {
            // Create variable which hold the value of ith path element of both agents without using auto
            std::pair<int, int> shorterAgentPathElement = *std::next(paths[shorterAgent].begin(), i);
            std::pair<int, int> longerAgentPathElement = *std::next(paths[longerAgent].begin(), i);
            // Check if the two agents are at the same location at the same timestep
            if (shorterAgentPathElement.first == longerAgentPathElement.first) 
            {
                conflicts.push_back(conflict(shorterAgent, longerAgent, shorterAgentPathElement.first, i));
            }
        }
        
    }
    return conflicts;
}

//create a function to get the list of edge conflicts
list<conflict> MAPFPlanner::findEdgeConflictsList(const std::vector<std::list<std::pair<int, int>>>& paths)
{
    // printf("Finding Edge Conflicts \n");
    std::vector<std::pair<int, int>> agentCombinations = generateCombinations(paths.size());
    // printf("Agent Combinations -> Size of agent list %ld \n",agentCombinations.size());
    list<conflict> conflicts;
    // Check conflicts for each combination of 2 agents
    for (const auto& combination : agentCombinations) {
        // In the combination, combination[0] is the first agent and combination[1] is the second agent
        int agent1 = combination.first;
        int agent2 = combination.second;

        // Find the agent with the shorter path
        int shorterAgent = (paths[agent1].size() < paths[agent2].size()) ? agent1 : agent2;
        int longerAgent = (shorterAgent == agent1) ? agent2 : agent1;
        // printf("Paths -> Shorted : %ld, Longer : %ld \n",paths[shorterAgent].size(),paths[longerAgent].size());
        if (paths[shorterAgent].size() == 0 || paths[longerAgent].size() == 0) 
        {
            continue;
        }

        if(env->curr_states[agent1].location == paths[agent1].front().first && env->curr_states[agent1].location == paths[agent2].front().first)
        {
            conflicts.push_back(conflict(agent1, agent2, env->curr_states[agent1].location, 0));
            continue;
        }
        else if(env->curr_states[agent2].location == paths[agent2].front().first && env->curr_states[agent2].location == paths[agent1].front().first)
        {
            conflicts.push_back(conflict(agent1, agent2, env->curr_states[agent2].location, 0));
            continue;
        }
        else if(env->curr_states[agent1].location == paths[agent2].front().first && env->curr_states[agent2].location == paths[agent1].front().first)
        {
            conflicts.push_back(conflict(agent1, agent2, env->curr_states[agent1].location, 0));
            continue;
        }
        // Run a loop of length equal to the length of the shorter path where ith element of both paths are checked if they are equal
        for (int i = 0; i < paths[shorterAgent].size()-1; i++) 
        {
            // Create variable which hold the value of ith and i+1th path element of both agents without using auto
            std::pair<int, int> shorterAgentPath_Current = *std::next(paths[shorterAgent].begin(), i);
            std::pair<int, int> shorterAgentPath_Next = *std::next(paths[shorterAgent].begin(), i+1);
            std::pair<int, int> longerAgentPath_Current = *std::next(paths[longerAgent].begin(), i);
            std::pair<int, int> longerAgentPath_Next = *std::next(paths[longerAgent].begin(), i+1);
            // Check if the two agents are at the same location at the same timestep
            if (shorterAgentPath_Current.first == longerAgentPath_Next.first && shorterAgentPath_Next.first == longerAgentPath_Current.first) 
            {
                conflicts.push_back(conflict(shorterAgent, longerAgent, shorterAgentPath_Current.first, longerAgentPath_Current.first, i));
            }
        }
    }

    return conflicts;
}

//create a function to get the first edge conflicts
conflict MAPFPlanner::findEdgeConflicts(const std::vector<std::list<std::pair<int, int>>>& paths) 
{
    std::vector<int> agentIndices(paths.size());
    std::vector<std::vector<int>> agentCombinations = combinations(agentIndices, 2);
    printf("Agent Combinations -> Size of agent list %ld \n",agentCombinations.size());

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
            if (shorterAgentPath_Current.first == longerAgentPath_Next.first && shorterAgentPath_Next.first == longerAgentPath_Current.first) 
            {
                return conflict(shorterAgent, longerAgent, shorterAgentPath_Current.first, longerAgentPath_Current.first, i);
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
vector<constraint_format> MAPFPlanner::convertToConstraint(conflict Conflict) 
{
    vector<constraint_format> constraints;
    if (Conflict.vertex2 == -1) // Check if the conflict is a vertex conflict
    {
        constraints.push_back(constraint_format(Conflict.agent1, Conflict.vertex1, Conflict.timestep));
        constraints.push_back(constraint_format(Conflict.agent2, Conflict.vertex1, Conflict.timestep));
    } 
    else // If the conflict is an edge conflict
    {
        constraints.push_back(constraint_format(Conflict.agent1, Conflict.vertex1, Conflict.vertex2, Conflict.timestep));
        constraints.push_back(constraint_format(Conflict.agent2, Conflict.vertex2, Conflict.vertex1, Conflict.timestep));
    }
    return constraints;
}

// I want a function like single_agent_plan which takes in a list of constraints and returns the path
list<pair<int, int>> MAPFPlanner::single_agent_plan_NT(int start, int start_direct, int end, vector<constraint_format> _constraints)
{
    list<pair<int, int>> path;
    priority_queue<AstarNode*, vector<AstarNode*>, cmp> open_list;
    unordered_map<int, AstarNode*> all_nodes;
    unordered_set<int> close_list;
    
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start, end), nullptr);
    open_list.push(s);
    all_nodes[start * 4 + start_direct] = s;

    // Create a vector with the with the location value of all the constraints
    vector<int> constraints;
    for (const auto& constraint : _constraints) 
    {
        constraints.push_back(constraint.vertex_1);
        if (constraint.vertex_2 != -1) 
        {
            constraints.push_back(constraint.vertex_2);
        }
    }

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location * 4 + curr->direction);

        if (curr->location == end)
        {
            while (curr->parent != NULL)
            {
                path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            break;
        }

        list<pair<int, int>> neighbors = getNeighbors(curr->location, curr->direction);

        for (const pair<int, int>& neighbor : neighbors)
        {
            int neighborKey = neighbor.first * 4 + neighbor.second;

            // Check if the neighbor is a constraint (obstacle)
            if (find(constraints.begin(), constraints.end(), neighbor.first) != constraints.end())
                continue;

            if (close_list.find(neighborKey) != close_list.end())
                continue;

            if (all_nodes.find(neighborKey) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighborKey];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g + 1;
                    old->f = old->h + old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                                                     curr->g + 1, getManhattanDistance(neighbor.first, end), curr);
                open_list.push(next_node);
                all_nodes[neighborKey] = next_node;
            }
        }
    }

    for (auto n : all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();

    return path;
}

// Write a function called plan which does the following
// 1. Find the path of all agents using single_agent_plan
// 2. Find the next steps for all agents 
// 3. Find the agents which will be in conflict when the action is taken. Build a list of there conflicting pairs.
// 4. Find the minimum list of agents that occur only once in all the conflicting pairs. This is the set of agents that need to be resolved.
// 5. Assign a waiting action to these agents. Replan the path for the remaining agent in the conflict pair with the waiting agent as a constraint.
// 6. Repeat steps 2-5 till there are no conflicts left.
// 7. Return the action list for all agents.
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    vector<constraint_format> init_constraints;
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    // Create a constraint list with the current state of the agents
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        init_constraints.push_back(constraint_format(i, env->curr_states[i].location, 0));
    }
    // Implement Step 1 here
    vector<list<pair<int,int>>> solution;
    for (int i = 0; i < env->num_of_agents; i++) 
    {   
        list<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else 
        {
            path = single_agent_plan_NT(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first, {});
        }
        solution.push_back(path);
    }
    // printf("Initial Solution Generated -> Size of solution vector %ld \n",solution.size());
    
    // Find the list of conflicts in the next step
    // Create an independent copy of solutions
    vector<list<pair<int,int>>> solution_copy = solution;

    list<conflict> conflicts = findVertexConflictsList(solution_copy);
    printf("Initial Conflicts Found -> Size of conflict list %ld \n",conflicts.size());
    // printf("VERTEX CONFLICTS: \n");    
    // for (const auto& conflict : conflicts) 
    // {
    //     printf("Agent %d and Agent %d at location %d at timestep %d \n",conflict.agent1,conflict.agent2,conflict.vertex1,conflict.timestep);
    // }
    conflicts.remove_if([](const conflict& c) { return c.timestep != 0; });
    printf("Updated Conflicts Found -> Size of conflict list %ld \n",conflicts.size());
    
    // Using the agents in the conflicts, find the minimum list of agents that occur only once in all the conflicting pairs
    // This is the set of agents that need to be resolved
    vector<int> agentsToResolve;
    for (const auto& conflict : conflicts) 
    {
        if (std::find(agentsToResolve.begin(), agentsToResolve.end(), conflict.agent1) == agentsToResolve.end()) 
        {
            agentsToResolve.push_back(conflict.agent1);
        }
        else if (std::find(agentsToResolve.begin(), agentsToResolve.end(), conflict.agent2) == agentsToResolve.end()) 
        {
            agentsToResolve.push_back(conflict.agent2);
        }
    }
    printf("Agents to resolve -> Size of agent list %ld \n",agentsToResolve.size());
    // printf("Agents to resolve: \n");

    // for (const auto& agent : agentsToResolve) 
    // {
    //     printf("Agent %d \n",agent);
    // }
    // Create a list of constraints for the agents to resolve and recalculate the path for the remaining agents
    vector<constraint_format> constraints;
    // Create an independent copy of solutions
    vector<list<pair<int,int>>> solution_copy2 = solution;
    for (const auto& agent : agentsToResolve) 
    {
        //add the current location as a constraint
        constraints.push_back(constraint_format(agent, env->curr_states[agent].location, 0));
        // constraints.push_back(constraint_format(agent, solution_copy2[agent].front().first, 0));
        // solution_copy2[agent].pop_front();
        // constraints.push_back(constraint_format(agent, solution_copy2[agent].front().first, 0));
    }
    printf("\n");
     vector<list<pair<int,int>>> solution_copy3 = solution;
    // Do the same for edge constraints here
    list<conflict> edgeConflicts = findEdgeConflictsList(solution_copy3);
    printf("Initial Edge Conflicts Found -> Size of conflict list %ld \n",edgeConflicts.size());
    // printf("EDGE CONFLICTS: \n");
    // for (const auto& conflict : edgeConflicts) 
    // {
    //     printf("Agent %d and Agent %d at location %d at timestep %d \n",conflict.agent1,conflict.agent2,conflict.vertex1,conflict.timestep);
    // }
    edgeConflicts.remove_if([](const conflict& c) { return c.timestep != 0; });
    printf("Updated Edge Conflicts Found -> Size of conflict list %ld \n",edgeConflicts.size());

    // Using the agents in the conflicts, find the minimum list of agents that occur only once in all the conflicting pairs
    // This is the set of agents that need to be resolved
    vector<int> agentsToResolveEdge;
    for (const auto& conflict : edgeConflicts) 
    {
        if (std::find(agentsToResolveEdge.begin(), agentsToResolveEdge.end(), conflict.agent1) == agentsToResolveEdge.end()) 
        {
            agentsToResolveEdge.push_back(conflict.agent1);
        }
        else if (std::find(agentsToResolveEdge.begin(), agentsToResolveEdge.end(), conflict.agent2) == agentsToResolveEdge.end()) 
        {
            agentsToResolveEdge.push_back(conflict.agent2);
        }
    }

    printf("Agents to resolve -> Size of agent list %ld \n",agentsToResolveEdge.size());
    // printf("Agents to resolve: \n");
    // for (const auto& agent : agentsToResolveEdge) 
    // {
    //     printf("Agent %d \n",agent);
    // }
    // Create a list of constraints for the agents to resolve and recalculate the path for the remaining agents
    vector<list<pair<int,int>>> solution_copy4 = solution;
    for (const auto& agent : agentsToResolveEdge) 
    {   
        // If agent not in agents to resolve
        if (std::find(agentsToResolve.begin(), agentsToResolve.end(), agent) == agentsToResolve.end())
        {
            constraints.push_back(constraint_format(agent, env->curr_states[agent].location, 0));
            constraints.push_back(constraint_format(agent, solution_copy2[agent].front().first, 0));
            solution_copy2[agent].pop_front();
            constraints.push_back(constraint_format(agent, solution_copy2[agent].front().first, 1));
            // solution_copy2[agent].pop_front();
            // constraints.push_back(constraint_format(agent, solution_copy2[agent].front().first, 2));
        }
        
    }
    printf("Constraints to resolve -> Size of constraint list %ld \n",constraints.size());
    // Print the locations of the constraints
    // printf("Constraints to resolve: \n");
    // for (const auto& constraint : constraints) 
    // {   
    //     // COnvert location from int to pair
    //     std::pair<int, int> location = convertToPair(constraint.vertex_1);
    //     printf("(%d, %d) \n",location.first,location.second);
    // }
    // Recalculate the path for the remaining agents
    vector<list<pair<int,int>>> final_solution;
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,int>> path;
        if (std::find(agentsToResolve.begin(), agentsToResolve.end(), i) == agentsToResolve.end()
            || std::find(agentsToResolveEdge.begin(), agentsToResolveEdge.end(), i) == agentsToResolveEdge.end())
        {
            //  printf("Replan for Agent %d \n",i);
            path = single_agent_plan_NT(env->curr_states[i].location,
                                            env->curr_states[i].orientation,
                                            env->goal_locations[i].front().first, constraints);
        }
        else 
        {
            path = solution[i];
        }
        final_solution.push_back(path);
    }
    // Print the solution of agent 0 in the solution vector in terms of lcoation pair
    // Get the location of agent 9 and convert from int to pair
    // std::pair<int, int> loc1 = convertToPair(env->curr_states[9].location);
    // printf("Solution of Agent 9 -> Size of solution list %ld | Current Location = %d, %d \n",final_solution[9].size(), loc1.first, loc1.second);
    // printf("Solution of Agent 9: \n");
    // for (const auto& location : final_solution[9]) 
    // {   
    //     // COnvert location from int to pair
    //     std::pair<int, int> locationpair = convertToPair(location.first);
    //     printf("(%d, %d) \n",locationpair.first,locationpair.second);
    // }
    // std::pair<int, int> loc2 = convertToPair(env->curr_states[16].location);
    // printf("Solution of Agent 16 -> Size of solution list %ld | Current Location = %d, %d \n",final_solution[16].size(), loc2.first, loc2.second);
    // printf("Solution of Agent 16: \n");
    // for (const auto& location : final_solution[16]) 
    // {   
    //     // COnvert location from int to pair
    //     std::pair<int, int> locationpair = convertToPair(location.first);
    //     printf("(%d, %d) \n",locationpair.first,locationpair.second);
    // }

    for (int i = 0; i < env->num_of_agents; i++) 
    {
        // printf("Agent %d -> Location %d, Orientation %d \n",i,env->curr_states[i].location,env->curr_states[i].orientation);
        list<pair<int,int>> path = final_solution[i];
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else 
        {
            path = final_solution[i];
        }
        if (path.front().first != env->curr_states[i].location)
        {
            // Check if forward action is possible, then assign forward action else assign wait action
            if (validateMove(path.front().first, env->curr_states[i].location))
            {
                // printf("Agent %d -> Action FW \n",i);
                actions[i] = Action::FW; //forward action
            }
            else
            {
                // printf("Agent %d -> Action W \n",i);
                actions[i] = Action::W; //wait action
            }
        } 
        else if (path.front().second!= env->curr_states[i].orientation)
        {
            int incr = path.front().second - env->curr_states[i].orientation;
            if (incr == 1 || incr == -3)
            {
                // printf("Agent %d -> Action CR \n",i);
                actions[i] = Action::CR; //C--counter clockwise rotate
            } 
            else if (incr == -1 || incr == 3)
            {
                // printf("Agent %d -> Action CCR \n",i);
                actions[i] = Action::CCR; //CCR--clockwise rotate
            } 
        }
    }

    printf("Initial Actions Assigned -> Removing Conflicting Agents\n\n\n");
    // Assign agents in the agents to resolve a waiting action
    //merge agents to resolve and agents to resolve edge such that there are no duplicates
    agentsToResolve.insert(agentsToResolve.end(), agentsToResolveEdge.begin(), agentsToResolveEdge.end());

    for (const auto& agent : agentsToResolve) 
    {
        actions[agent] = Action::W;
    }
    // Return the set of actions
    return;
}


