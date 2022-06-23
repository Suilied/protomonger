#pragma once

#include <vector>
#include "Vector2.h"

class AgentPlan {
private:
	std::vector<int> agents;
	std::vector<Vector2> route;

	friend class AgentPlanner;
};

class AgentPlanner {
private:
	std::vector<AgentPlan> plans;

	std::vector<int> agents;

public:
	void new_plan();
	friend class AgentPlan;
};