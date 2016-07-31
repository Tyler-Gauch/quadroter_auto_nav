#include "Problem.h"

struct SetCompare{
	bool operator() (const State& lhs, const State& rhs) const{
		if(lhs.x == rhs.x)
		{
			return lhs.y < rhs.y;
		}
        return lhs.x < rhs.x;
    }
};

Problem::Problem(ros::NodeHandle &nh, nav_msgs::OccupancyGrid::ConstPtr map){
	this->debug = new Debugger(nh, "States_Expanded", 0,1,0);
	// this->debug->turnOff();
	this->map = map;
	this->successorOffset = 4;
}

Problem::~Problem(){
	delete debug;
}

bool Problem::isGoalState(State state){
	if(this->goalStates.size() > 0)
	{
		for(int i = 0; i < this->goalStates.size(); i++)
		{
			if(CommonUtils::getDistance(this->goalStates.at(i).x, this->goalStates.at(i).y, state.x, state.y) <= this->successorOffset)
			{
				return true;
			}
		}
		return false;
	}

	return (state.value == -1);
}

//currently only did N,S,E,W can do NE,NW,SE,SW after
std::vector<State> Problem::getSuccessors(State state){
	std::vector<State> successors;
	if(state.y > this->successorOffset)
	{
		//get the immediatly next north grid point
		int northIndex = CommonUtils::getIndex(state.x, state.y+this->successorOffset, map);
		if(map->data[northIndex] <= 0)
		{
			State north(state.x, state.y+this->successorOffset, map->data[northIndex]);
			north.cost = state.cost+1;
			successors.push_back(north);
		}
	}
	if(state.y < map->info.height - this->successorOffset)
	{
		//gets the immeditaly next south grid point
		int southIndex = CommonUtils::getIndex(state.x, state.y-this->successorOffset, map);
		if(map->data[southIndex] <= 0)
		{
			State south(state.x, state.y-this->successorOffset, map->data[southIndex]);
			south.cost = state.cost+1;
			successors.push_back(south);
		}
	}
	if(state.x < map->info.width - this->successorOffset)
	{
		//gets the immeditaly next east grid point
		int eastIndex = CommonUtils::getIndex(state.x+this->successorOffset, state.y, map);
		if(map->data[eastIndex] <= 0)
		{
			State east(state.x+this->successorOffset, state.y, map->data[eastIndex]);
			east.cost = state.cost+1;
			successors.push_back(east);
		}
	}
	if(state.x > this->successorOffset)
	{
		//gets the immeditaly next west grid point
		int westIndex = CommonUtils::getIndex(state.x-this->successorOffset, state.y, map);
		if(map->data[westIndex] <= 0)
		{
			State west(state.x-this->successorOffset, state.y, map->data[westIndex]);
			west.cost = state.cost+1;
			successors.push_back(west);
		}
	}
	if(state.x > this->successorOffset && state.y > this->successorOffset)
	{
		//gets the immeditaly next north west grid point
		int northWestIndex = CommonUtils::getIndex(state.x-this->successorOffset, state.y+this->successorOffset, map);
		if(map->data[northWestIndex] <= 0)
		{
			State northWest(state.x-this->successorOffset, state.y+this->successorOffset, map->data[northWestIndex]);
			northWest.cost = state.cost+0.4 + 1;
			successors.push_back(northWest);
		}
	}
	if(state.x > this->successorOffset && state.y < map->info.height - this->successorOffset)
	{
		//gets the immeditaly next south west grid point
		int southWestIndex = CommonUtils::getIndex(state.x-this->successorOffset, state.y-this->successorOffset, map);
		if(map->data[southWestIndex] <= 0)
		{
			State southWest(state.x-this->successorOffset, state.y-this->successorOffset, map->data[southWestIndex]);
			southWest.cost = state.cost+0.4 + 1;
			successors.push_back(southWest);
		}
	}
	if(state.x < map->info.width - this->successorOffset && state.y > this->successorOffset)
	{
		//gets the immeditaly next north east grid point
		int northEastIndex = CommonUtils::getIndex(state.x+this->successorOffset, state.y+this->successorOffset, map);
		if(map->data[northEastIndex] <= 0)
		{
			State northEast(state.x+this->successorOffset, state.y+this->successorOffset, map->data[northEastIndex]);
			northEast.cost = state.cost+0.4 + 1;
			successors.push_back(northEast);
		}
	}
	if(state.x < map->info.width - this->successorOffset && state.y < map->info.height - this->successorOffset)
	{
		//gets the immeditaly next south east grid point
		int southEastIndex = CommonUtils::getIndex(state.x+this->successorOffset, state.y-this->successorOffset, map);
		if(map->data[southEastIndex] <= 0)
		{
			State southEast(state.x+this->successorOffset, state.y-this->successorOffset, map->data[southEastIndex]);
			southEast.cost = state.cost+0.4 + 1;
			successors.push_back(southEast);
		}
	}
	return successors;
}

int Problem::heuristic(State& state){
	int score = 0;
	score += this->checkStateForObstacle(state);
	if(this->goalStates.size() > 0)
	{
		int smallestDistance = 100;
		for(int i = 0; i < goalStates.size(); i++)
		{
			smallestDistance = std::min((int)CommonUtils::getDistance(state.x, state.y, this->goalStates.at(i).x, this->goalStates.at(i).y), smallestDistance);
		}
		score += smallestDistance;
	}
	return score;
}

int Problem::checkStateForObstacle(State& state){
	int threshold = 9;
//	int obstacleThreshold = 15;
	int score = 0;

	for(int y = -threshold; y < threshold; y++)
	{
		for(int x = -threshold; x < threshold; x++)
		{
			if(this->map->data[CommonUtils::getIndex(state.x+x, state.y+y, this->map)] > 0)
			{
				state.obstacle = true;
//				if (std::max(abs(y), abs(x)) <= obstacleThreshold) {
					score += 1000000/(std::max(abs(y), abs(x))+1);
//				}
			}
		}
	}
	return score;
}

std::vector<State> Problem::search(State startState, State goalState){
	this->goalStates.clear();
	this->goalStates.push_back(goalState);
	return this->search(startState);
}

std::vector<State> Problem::search(State startState){

	std::cout << "Attempting search..." << std::endl;
	std::set<State, SetCompare> closedList;  //used the SetCompare to compare the x,y 

	std::priority_queue<State> frontier;

	frontier.push(startState);

	debug->removePoints();

	while(!frontier.empty())
	{
		State s = frontier.top(); //views top node
		State * state = new State();
		state->x = s.x;
		state->y = s.y;
		state->cost = s.cost;
		state->parent = s.parent;
		state->value = s.value;
		state->priority = s.priority;
		frontier.pop(); //removes top node
	
		// std::cout << "Processing state at (" << state->x << ", " << state->y << ") Priority: " << state->priority << " Value: " << state->value << std::endl;
		if(this->isGoalState(*state))
		{
			std::vector<State> path;

			// std::cout << "Found goal (" << state->x << ", " << state->y << ") Parent: " << state->parent << std::endl;

			State * temp = state;

			if(temp->parent == NULL)
			{
				this->goalStates.clear();
				path.push_back(*temp);
				return path;
			}

			while(temp->parent != NULL)
			{
				// std::cout << "Adding (" << temp->x << ", " << temp->y << ") to path. COST: " << temp->priority << std::endl;
				path.push_back(*temp);
				temp = temp->parent;
			}


			return path;
		}else if(closedList.find(*state) == closedList.end()){
			closedList.insert(*state); 
			std::vector<State> successors = this->getSuccessors(*state);
			debug->addPoint(CommonUtils::getTransformXPoint(state->x, map), CommonUtils::getTransformYPoint(state->y, map), 0);
			for(int i = 0; i < successors.size(); i++)
			{
				State successor = successors[i];
				int heuristic = this->heuristic(successor);
				if(heuristic > 100000)
				{
					continue;
				}
				successor.priority = successor.cost + heuristic;

				successor.parent = state;
				// std::cout << "\tSuccessor: (" << successor.x << ", " << successor.y << ") Priority: " << successor.priority << " Parent: " << successor.parent << std::endl;
				frontier.push(successor);
			}
		}

		debug->publishPoints();
	}

	this->goalStates.clear();
	std::vector<State> path;
	return path;//no path 
}