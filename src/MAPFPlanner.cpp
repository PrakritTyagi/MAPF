#include <MAPFPlanner.h>
#include <random>
#include<vector>

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

struct constraint_format{
    public:
    int agent_id,t, vertex_1, vertex_2;
    constraint_format(int _agent_id, int _vertex_1, int _vertex_2, int t): agent_id(_agent_id), vertex_1(_vertex_1), vertex_2(_vertex_2), t(t) {}
    constraint_format(int _agent_id, int _vertex_1, int t): agent_id(_agent_id), vertex_1(_vertex_1), t(t) {}
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
