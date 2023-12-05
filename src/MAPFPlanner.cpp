#include <MAPFPlanner.h>
#include <random>

// vector<list<pair<int,int>>> CBS_solution;

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
void MAPFPlanner::naive_CBS(vector<bool> find_new_path)
{   
    // create a root node with empty constraint variable
    std::shared_ptr<CT_node> root_node = std::make_shared<CT_node>();
    // calculate paths for all agents using A*(TODO: heuristics are already calculated in initialize function)
    // store it the root node
    // unordered_map<int, std::shared_ptr<CT_node> > all_nodes;
    vector<list<pair<int,int>>> solution;
    // all_nodes[0]=root_node;

    for (int i = 0; i < env->num_of_agents; i++) 
    {   
        list<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else 
        {   
            if(find_new_path[i]){
                path = single_agent_plan(i,env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first, root_node->node_constraints);
            }
            else{
                pair<int,int> curr_state=make_pair(env->curr_states[i].location, env->curr_states[i].orientation);
                auto it=find(carry_forward_soln[i].begin(),carry_forward_soln[i].end(),curr_state);
                
                if(it!=carry_forward_soln[i].end()){
                    path = list<pair<int,int>>(it,carry_forward_soln[i].end());
                }
                else{
                    path = single_agent_plan(i,env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first, root_node->node_constraints);
                }
            }
            
        }
        solution.push_back(path);
    }
    root_node->node_solution = solution;
   
    // calculate sum-of-cost and store it in root node
    root_node->SOC = sum_of_costs(root_node->node_solution);

    priority_queue<std::shared_ptr<CT_node>,vector<std::shared_ptr<CT_node>>,CT_CMP> OPEN_LIST;
    OPEN_LIST.push(root_node);
    // int counter=0;
    // while loop till open_list is empty
    while (!OPEN_LIST.empty())
    {   
        // counter++;
        // cout<<"1"<<endl;
        // pop the best CT_node with lowest sum-of-cost (create a comparator function to compare CT_nodes)
        std::shared_ptr<CT_node> curr_node = OPEN_LIST.top();
        
        OPEN_LIST.pop();

        // check for conflicts in the paths (create a conflict finding function)
        conflict final_conflict = findCardinalConflicts(curr_node->node_solution, curr_node->node_constraints);
    
        // if no conflict, return the solution as this is the goal
 
        if (final_conflict.agent1 == -1 && final_conflict.agent2 == -1) 
        {
            final_conflict = findConflicts(curr_node->node_solution, curr_node->node_constraints);
            if(final_conflict.agent1 == -1 && final_conflict.agent2 == -1){
                // right code to delete all nodes
               
                // return the solution
                cout << "Solution found" << endl;
                CBS_solution = curr_node->node_solution;
                carry_forward_soln = curr_node->node_solution;
                //  for (auto n: all_nodes)
                // {
                //     // delete n.second;
                // }
                return ;
            }
        }
            
        
        // if conflict, create two new CT_nodes
        // first node
        vector<constraint_format> current_constraint_stack = convertToConstraint(final_conflict);

        std::shared_ptr<CT_node> left_node = std::make_shared<CT_node>();
        // all_nodes[counter]=left_node;
        // counter++;
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
        //  push the  new CT_node in open_list
        if(left_node->SOC < INT64_MAX && !new_path.empty())
            OPEN_LIST.push(left_node);
        // second node
        std::shared_ptr<CT_node> right_node = std::make_shared<CT_node>();
        // all_nodes[counter]=right_node;
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

        // push the new CT_node in open_list
        
        if(right_node->SOC < INT64_MAX && !new_path.empty())    
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

/**
 * @fn computerHeuristic
 * @brief Compute heuristic for all paths
*/
void MAPFPlanner::computeHeuristic()
{
    // get all start locations in a vector
    vector<AstarNode*> all_start_locations;
    // get map size
    int map_size = env->map.size();

    for(int i=0; i < map_size; i++)
    {   if(validateCell(i)){
            for(int j=0; j < 4; j++)
            {
                AstarNode* s = new AstarNode(i, j, 0, 0, 0, nullptr); // location id, direction, g, h, t, parent
                all_start_locations.push_back(s);
            }
        }
    }

    int count = 0;
    for(auto start_loc:all_start_locations)
    {
        
        priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
        unordered_map<int,AstarNode*> all_nodes;
        unordered_map<int, int>  heuristics; // key is location*4+direction, pair is g and time
        open_list.push(start_loc);
        all_nodes[start_loc->location*4 + start_loc->direction] = start_loc;
        
        while (!open_list.empty())
        {
            AstarNode* curr = open_list.top();
            open_list.pop();
            if(heuristics.find(curr->location*4 + curr->direction) != heuristics.end())
            {
                continue;
            }
            heuristics[curr->location*4 + curr->direction] = curr->g;

            list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
            for (const pair<int,int>& neighbor: neighbors)
            {   
                if (heuristics.find(neighbor.first*4 + neighbor.second) != heuristics.end())
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
                    AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second, curr->g+1, 0, 0, curr);
                    open_list.push(next_node);
                    all_nodes[neighbor.first*4+neighbor.second] = next_node;
                }
            }
        }
        
        heuristics_map[start_loc->location*4 + start_loc->direction] = heuristics;
        for (auto n: all_nodes)
        {
            delete n.second;
        }
        all_nodes.clear();
        count++;
    }
    // print heuristic size
    cout << "heuristic map size: " << heuristics_map.size() << endl;
    // print heuristic for a first key
    cout << "heuristic for first key: " << endl;
    // get first heuristic from heuristics_map
    auto first_heuristic = heuristics_map[20];
    // print first heuristic size
    cout << "first heuristic size: " << first_heuristic.size() << endl;
    // print first_heuristic
    for(auto h: first_heuristic)
    {
        cout << "location*4+direction: " << h.first << " g: " << h.second << endl;
    }
    // print map row and col
    cout << "map row: " << env->rows << " map col: " << env->cols << endl;
}

/**
 * @fn get_heuristic
 * @brief Get heuristic for a given location and direction
*/
int MAPFPlanner::get_heuristic(int start_loc, int start_dir, int goal_loc)
{
    // get heuristic for a given location and direction
    auto heuristic = heuristics_map[start_loc*4 + start_dir];
    // get heuristic for a given goal location and all directions and take the minimum
    int min_heuristic = INT_MAX;
    for(int i=0; i < 4; i++)
    {
        int heuristic_value = heuristic[goal_loc*4 + i];
        if(heuristic_value < min_heuristic)
        {
            min_heuristic = heuristic_value;
        }
    }

    return min_heuristic;
}

/**
 * @fn initialize
 * @brief Initialize the planner
*/
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    cout << "****************************************************" << endl;
    cout << "planner preprocess time limit: " << preprocess_time_limit << endl;
    // start timer
    // auto start = std::chrono::high_resolution_clock::now();
    // computeHeuristic();
    // // end timer
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
    // cout << "Heuristic compute time: " << duration.count() << "milliseconds" <<endl;  

    // calculate heuristic for all agents and store
    cout << "planner initialize done" << endl;
    // exit(1);
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
    static vector<bool> find_new_path(env->num_of_agents,true);
    if(run_cbs){
        
        cout<<"Running CBS"<<endl;
        naive_CBS(find_new_path);
        run_cbs=false;
        cout<<"CBS done"<<endl;
        
        for(int i=0;i<find_new_path.size();i++){
            find_new_path[i]=false;
        }

        // for(int i=0;i<CBS_solution.size();i++){
        //     CBS_solution[i].pop_front();
        // }
    }
    
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    
    for (int i = 0; i < env->num_of_agents; i++) 
    {   int count=0;
        list<pair<int,int>> path;
        if (env->goal_locations[i].empty()) 
        {
            cout<<"Completed All the Goals!!!!!!!!!!!!"<<endl;
            path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        }
        else 
        {   
            path = CBS_solution[i];
            
            
            // If About to reach goal in next step then run CBS again
            if(path.front().first==env->goal_locations[i].front().first ){
                run_cbs=true;
                find_new_path[i]=true;
            }
            
            if(!CBS_solution[i].empty()){
                CBS_solution[i].pop_front();
            }
            else{     
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
                    if(n.vertex_1 ==node->location && n.t==node->t)
                        return true; 
                }

                // If Edge Constraint
                else{ 
                    if((n.vertex_2 ==node->location && n.t+1==node->t))  
                        return true;
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
    // AstarNode* s = new AstarNode(start, start_direct, 0, get_heuristic(start, start_direct, end), nullptr);
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
        {   AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,curr->g+1,getEuclideanDistance(neighbor.first, end),curr->t+1 ,curr);
            
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

bool MAPFPlanner::validateCell(int loc)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;
    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;
    return true;
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

bool MAPFPlanner::isCardinal(const vector<list<pair<int, int>>>& paths, conflict Conflict, vector<constraint_format>& constraints){
    vector<constraint_format> current_constraint_stack = convertToConstraint(Conflict);
    int agent1 = Conflict.agent1, agent2 = Conflict.agent2;
    vector<constraint_format> agent1_constraints=constraints, agent2_constraints=constraints;
    agent1_constraints.push_back(current_constraint_stack[0]);
    agent2_constraints.push_back(current_constraint_stack[1]);
    list<pair<int,int>> new_path = single_agent_plan(agent1,env->curr_states[agent1].location,
                                env->curr_states[agent1].orientation,
                                env->goal_locations[agent1].front().first, agent1_constraints);
    if(new_path.size()>paths[agent1].size()){
        new_path = single_agent_plan(agent2,env->curr_states[agent2].location,
                                env->curr_states[agent2].orientation,
                                env->goal_locations[agent2].front().first, agent2_constraints);
        if(new_path.size()>paths[agent2].size()){
            return true;
        }
    }
    return false;
    
    
}

conflict MAPFPlanner::findCardinalConflicts(const vector<list<pair<int, int>>>& paths, vector<constraint_format>& constraints) 
{
    vector<int> agentIndices(paths.size());
    iota(agentIndices.begin(), agentIndices.end(), 0);  // Fill with 0, 1, ..., n-1
    vector<std::vector<int>> agentCombinations = combinations(agentIndices);
    
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
        
        if(paths[shorterAgent].size()==0){
            continue;
        }

        // Run a loop of length equal to the length of the shorter path where ith element of both paths are checked if they are equal
        for (int i = 0; i < paths[shorterAgent].size(); i++) 
        {
            // Create variable which hold the value of ith path element of both agents without using auto
            
            pair<int, int> shorterAgentPath_Current = *std::next(paths[shorterAgent].begin(), i);
            pair<int, int> longerAgentPath_Current = *std::next(paths[longerAgent].begin(), i);
            // Check if the two agents are at the same location at the same timestep
            if (shorterAgentPath_Current.first == longerAgentPath_Current.first) 
            {   
                if(isCardinal(paths, conflict(shorterAgent, longerAgent, shorterAgentPath_Current.first, i), constraints)){
                    cout<<"Cardinal Conflict"<<endl;
                    cout<<"Vertex Conflict"<<endl;
                    cout<<"Agent 1: "<<shorterAgent<<" Agent 2: "<<longerAgent<<" Vertex: "<<shorterAgentPath_Current.first<<" TimeStep: "<<i<<endl;
                    return conflict(shorterAgent, longerAgent, shorterAgentPath_Current.first, i);
                }
            }
   
        }

    }
    
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
        
        if(paths[shorterAgent].size()==0){
            continue;
        }

        // Run a loop of length equal to the length of the shorter path where ith element of both paths are checked if they are equal
        for (int i = 0; i < paths[shorterAgent].size(); i++) 
        {
            // Create variable which hold the value of ith path element of both agents without using auto
            
            pair<int, int> shorterAgentPath_Current = *std::next(paths[shorterAgent].begin(), i);
            pair<int, int> shorterAgentPath_Next = *std::next(paths[shorterAgent].begin(), i+1);
            pair<int, int> longerAgentPath_Current = *std::next(paths[longerAgent].begin(), i);
            pair<int, int> longerAgentPath_Next = *std::next(paths[longerAgent].begin(), i+1);
            // Check if the two agents are at the same location at the same timestep
            
            if (shorterAgentPath_Current.first == longerAgentPath_Next.first && shorterAgentPath_Next.first == longerAgentPath_Current.first) 
            {   
                if(isCardinal(paths, conflict(shorterAgent, longerAgent, shorterAgentPath_Current.first, i), constraints)){
                    cout<<"Cardinal Conflict"<<endl;
                    cout<<"Edge Conflict"<<endl;
                    cout<<"Agent 1: "<<shorterAgent<<" Agent 2: "<<longerAgent<<" Vertex: "<<shorterAgentPath_Current.first<<" TimeStep: "<<i<<endl;
                    return conflict(shorterAgent, longerAgent, shorterAgentPath_Current.first, longerAgentPath_Current.first, i);
                }
            }


        }

    }
    return conflict();
}

conflict MAPFPlanner::findConflicts(const vector<list<pair<int, int>>>& paths, vector<constraint_format>& constraints) 
{
    vector<int> agentIndices(paths.size());
    iota(agentIndices.begin(), agentIndices.end(), 0);  // Fill with 0, 1, ..., n-1
    vector<std::vector<int>> agentCombinations = combinations(agentIndices);
    
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
        
        if(paths[shorterAgent].size()==0){
            continue;
        }

        // Run a loop of length equal to the length of the shorter path where ith element of both paths are checked if they are equal
        for (int i = 0; i < paths[shorterAgent].size(); i++) 
        {
            // Create variable which hold the value of ith path element of both agents without using auto
            
            pair<int, int> shorterAgentPath_Current = *std::next(paths[shorterAgent].begin(), i);
            pair<int, int> longerAgentPath_Current = *std::next(paths[longerAgent].begin(), i);
            // Check if the two agents are at the same location at the same timestep
            if (shorterAgentPath_Current.first == longerAgentPath_Current.first) 
            {   
                
                cout<<"Non Cardinal Conflict"<<endl;
                cout<<"Vertex Conflict"<<endl;
                cout<<"Agent 1: "<<shorterAgent<<" Agent 2: "<<longerAgent<<" Vertex: "<<shorterAgentPath_Current.first<<" TimeStep: "<<i<<endl;
                return conflict(shorterAgent, longerAgent, shorterAgentPath_Current.first, i);
                
            }
   
        }

    }
    
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
        
        if(paths[shorterAgent].size()==0){
            continue;
        }

        // Run a loop of length equal to the length of the shorter path where ith element of both paths are checked if they are equal
        for (int i = 0; i < paths[shorterAgent].size(); i++) 
        {
            // Create variable which hold the value of ith path element of both agents without using auto
            
            pair<int, int> shorterAgentPath_Current = *std::next(paths[shorterAgent].begin(), i);
            pair<int, int> shorterAgentPath_Next = *std::next(paths[shorterAgent].begin(), i+1);
            pair<int, int> longerAgentPath_Current = *std::next(paths[longerAgent].begin(), i);
            pair<int, int> longerAgentPath_Next = *std::next(paths[longerAgent].begin(), i+1);
            // Check if the two agents are at the same location at the same timestep
            
            if (shorterAgentPath_Current.first == longerAgentPath_Next.first && shorterAgentPath_Next.first == longerAgentPath_Current.first) 
            {   
                
                cout<<"Non Cardinal Conflict"<<endl;
                cout<<"Edge Conflict"<<endl;
                cout<<"Agent 1: "<<shorterAgent<<" Agent 2: "<<longerAgent<<" Vertex: "<<shorterAgentPath_Current.first<<" TimeStep: "<<i<<endl;
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

