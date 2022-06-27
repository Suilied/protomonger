#pragma once

#include <vector>
#include "Vector2.h"
#include "scribe.h"
#include "circlepacker.h"
#include "RVO.h"

struct AgentGoal {
    int agentId;
    float x;
    float y;
    float radius; // for visualisation (can be got from agent vector)
};

struct Waypoint {
    float x;
    float y;
    float radius;
    float radius_min;
    float radius_max;
    float radius_growth;
};

struct AgentPlan { // AgentGroup x table
    std::vector<RVO::Agent*> agents;
    std::vector<Vector2> agent_goals;
};

class AgentManager {
private:
	std::vector<Waypoint*> _path;
    RVO::RVOSimulator* _rvoSim;
    CirclePacker* _circlePacker;
    std::vector<Vector2> _agent_goals;

    /*
    * agents that have received a movement order will be put in a plan-group
    * agents that have been selected are in the _selected_agents vector.
    */
    std::vector<std::pair<bool, int>> _selected_agents;
    std::vector<AgentPlan*> _agent_plans;

    bool rvof_reached_goals();
    void update_rvof_velocities();

    // debug draw flags
    bool renderKdTree = true;

public:
    void init();
    void update(float deltaTime);
	void new_waypoint(float x, float y);
    void clear_waypoints();
    void debug_draw(Scribe* scribe);

    void set_agent_target(float x, float y);
    void spawn_agent(Vector2 position, Vector2 goal = Vector2(), bool selected = false);
    void spawn_agents_circular(Vector2 center_vec, int agent_count);
    void spawn_agents_square(Vector2 center_vec, int agent_count);

    void select_agent_point(float x, float y);
    void select_agent_box(float x0, float y0, float x1, float y1);

    AgentManager();
    ~AgentManager();
};