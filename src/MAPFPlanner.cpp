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

    for (int i = 0; i < env->num_of_agents; i++) 
    {   
        list<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else 
        {   
            path = single_agent_plan(i,env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first, root_node->node_constraints);
        }
        solution.push_back(path);
    }
    root_node->node_solution = solution;
   
    // calculate sum-of-cost and store it in root node
    root_node->SOC = sum_of_costs(root_node->node_solution);

    priority_queue<std::shared_ptr<CT_node>,vector<std::shared_ptr<CT_node>>,CT_CMP> OPEN_LIST;
    OPEN_LIST.push(root_node);
    
    // while loop till open_list is empty
    while (!OPEN_LIST.empty())
    {   
        // pop the best CT_node with lowest sum-of-cost (create a comparator function to compare CT_nodes)
        std::shared_ptr<CT_node> curr_node = OPEN_LIST.top();
        
        OPEN_LIST.pop();

        // check for conflicts in the paths (create a conflict finding function)
        conflict vertex_conflict = findVertexConflicts(curr_node->node_solution);
        conflict edge_conflict, final_conflict;
        final_conflict=vertex_conflict;
        // if no conflict, return the solution as this is the goal
        if (vertex_conflict.agent1 == -1 && vertex_conflict.agent2 == -1) 
        {   
            edge_conflict = findEdgeConflicts(curr_node->node_solution);
            final_conflict=edge_conflict;
            if (edge_conflict.agent1 == -1 && edge_conflict.agent2 == -1) 
            {
                // return the solution
                cout << "Solution found" << endl;
                CBS_solution = curr_node->node_solution;
                return ;
            }
            else{
                // cout<<"Edge Conflict"<<endl;
                // cout<<"Agent 1: "<<edge_conflict.agent1<<" Agent 2: "<<edge_conflict.agent2<<" Vertex: "<<edge_conflict.vertex1<<" TimeStep: "<<edge_conflict.timestep;
                // cout<<endl;
            }
        }
        else{
            // cout<<"Vertex Conflict"<<endl;
            // cout<<"Agent 1: "<<vertex_conflict.agent1<<" Agent 2: "<<vertex_conflict.agent2<<" Vertex: "<<vertex_conflict.vertex1<<" TimeStep: "<<vertex_conflict.timestep;
            // cout<<endl;
        }
        
        // if conflict, create two new CT_nodes
        // first node
        vector<constraint_format> current_constraint_stack = convertToConstraint(final_conflict);

        std::shared_ptr<CT_node> left_node = std::make_shared<CT_node>();
        curr_node->left_ptr = left_node;
        left_node->node_constraints = curr_node->node_constraints;
        left_node->node_constraints.push_back(current_constraint_stack[0]);
        left_node->node_solution = curr_node->node_solution;
        int agent1 = final_conflict.agent1, agent2 = final_conflict.agent2;
        list<pair<int,int>> new_path = single_agent_plan(agent1,env->curr_states[agent1].location,
                                    env->curr_states[agent1].orientation,
                                    env->goal_locations[agent1].front().first, left_node->node_constraints);
        left_node->node_solution[agent1] = new_path;
        left_node->SOC = sum_of_costs(left_node->node_solution);
        left_node->parent_ptr = curr_node;

        // second node
        std::shared_ptr<CT_node> right_node = std::make_shared<CT_node>();
        curr_node->right_ptr = right_node;
        right_node->node_constraints = curr_node->node_constraints;
        right_node->node_constraints.push_back(current_constraint_stack[1]);
        right_node->node_solution = curr_node->node_solution;
        new_path = single_agent_plan(agent2, env->curr_states[agent2].location,
                                    env->curr_states[agent2].orientation,
                                    env->goal_locations[agent2].front().first,right_node->node_constraints);
        right_node->node_solution[agent2] = new_path;
        right_node->SOC = sum_of_costs(right_node->node_solution);
        right_node->parent_ptr = curr_node;

        // push the two new CT_nodes in open_list
        if(left_node->SOC < INT64_MAX)
            OPEN_LIST.push(left_node);
        if(right_node->SOC < INT64_MAX)
            OPEN_LIST.push(right_node);
        
        curr_node.reset();
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
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
 {  
     
    static bool run_cbs=true;
    // static  vector<std::vector<std::pair<int, int>>> g_locations;
    static vector<list<pair<int,int>>> Solution;
    
    if(run_cbs){
        
        cout<<"Running CBS"<<endl;
        naive_CBS() ;
        run_cbs=false;
        cout<<"CBS done"<<endl;

    }
    
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    
    for (int i = 0; i < env->num_of_agents; i++) 
    {  
        list<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            cout<<"Completed All the Goals"<<endl;
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        }
        else 
        {   
            // cout<<"Alotted Path"<<endl;
            path = CBS_solution[i];
            // If About to reach goal in next step then run CBS again
            if(path.front().first==env->goal_locations[i].front().first ){
                run_cbs=true;
            }

            // cout<<"AgentId: "<<i<<endl;
            if(!CBS_solution[i].empty()){
                CBS_solution[i].pop_front();
            }
            else{
                // cout<<"Empty Path Agent: "<<i<<endl;
                path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
            }
               
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
        else{
            actions[i]=Action::W;
        }
        
        
    }
    

  return;
}

bool MAPFPlanner::found_node(int agent_id, vector<constraint_format> constraints, AstarNode* node){
    if(!constraints.empty()){
        for(constraint_format n: constraints){
            if(n.agent_id== agent_id){
            // If Vertex Constraint
                if(n.vertex_2==-1){
                    // if(n.vertex_1==941 && n.t==4){
                    //     cout<<"Node Location: "<<node->location<<"Time Step: "<<node->t<<endl;
                    // }
                    if(n.vertex_1 ==node->location && n.t==node->t){
                        // cout<<"Found Node: "<< " Vertex: "<< n.vertex_1<<" TimeStep: "<<n.t;
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
        }
    }
    return false;
}

list<pair<int,int>> MAPFPlanner::single_agent_plan(int agent_id, int start,int start_direct,int end, vector<constraint_format> constraints)
{
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getEuclideanDistance(start,end), nullptr);
    s->t=0;
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;
    
    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        
        if (curr->location == end )
        {   
            // cout<<"Goal Reached"<<endl;
            while(curr->parent!=NULL) 
            {
                path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            path.emplace_front(make_pair(curr->location, curr->direction));
            break;
        }

        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
        for (const pair<int,int>& neighbor: neighbors)
        {   AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,curr->g+1,getEuclideanDistance(neighbor.first,end),curr->t+1 ,curr);
            
            if(found_node(agent_id, constraints, next_node)){
                delete next_node;
                continue;
            }
            
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {   
                delete next_node;
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
        
                {   
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                    old->t=curr->t+1;
                }
            }
            else
            {    
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


int MAPFPlanner::getEuclideanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return sqrt(pow(loc1_x - loc2_x,2) + pow(loc1_y - loc2_y,2));
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

std::vector<std::vector<int>> MAPFPlanner::combinations(std::vector<int> arr) {
    std::vector<std::vector<int>> result;

    for (int i = 0; i < arr.size(); ++i) {
        for (int j = i + 1; j < arr.size(); ++j) {
            result.push_back({arr[i], arr[j]});
        }
    }

    return result;
}

conflict MAPFPlanner::findVertexConflicts(const std::vector<std::list<std::pair<int, int>>>& paths) 
{
    std::vector<int> agentIndices(paths.size());
    std::iota(agentIndices.begin(), agentIndices.end(), 0);  // Fill with 0, 1, ..., n-1
    std::vector<std::vector<int>> agentCombinations = combinations(agentIndices);
    
    // Check conflicts for each combination of 2 agents
    for (const auto& combination : agentCombinations) {
        // In the combination, combination[0] is the first agent and combination[1] is the second agent
        if(combination[0]==combination[1]){
            continue;
        }
       
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

//create a function to get the first edge conflicts
conflict MAPFPlanner::findEdgeConflicts(const std::vector<std::list<std::pair<int, int>>>& paths) 
{
    std::vector<int> agentIndices(paths.size());
    std::iota(agentIndices.begin(), agentIndices.end(), 0); 
    std::vector<std::vector<int>> agentCombinations = combinations(agentIndices);
  
    // Check conflicts for each combination of 2 agents
    
    for (const auto& combination : agentCombinations) {
        // In the combination, combination[0] is the first agent and combination[1] is the second agent
        if(combination[0]==combination[1]){
            
            continue;
        }
      
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
          
            // if((agent1==3 && agent2==5)){
            //     cout<<"Agent 1: "<<agent1<<" Agent 2: "<<agent2<<endl;
            //     cout<<"Shorter Agent Path Current: "<<shorterAgentPath_Current.first<<" Shorter Agent Path Next: "<<shorterAgentPath_Next.first<<endl;
            //     cout<<"Longer Agent Path Current: "<<longerAgentPath_Current.first<<" Longer Agent Path Next: "<<longerAgentPath_Next.first<<endl;
            // }

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
        constraints.push_back(constraint_format(Conflict.agent2, Conflict.vertex1, Conflict.vertex2, Conflict.timestep));
    }
    return constraints;
}

