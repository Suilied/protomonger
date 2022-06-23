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

class AgentManager {
private:
	std::vector<Waypoint*> _path;
    RVO::RVOSimulator* _rvoSim;
    CirclePacker* _circlePacker;
    std::vector<Vector2> _agent_goals;

    bool rvof_reached_goals();
    void update_rvof_velocities();

public:
    void init();
    void update(float deltaTime);
	void new_waypoint(float x, float y);
    void clear_waypoints();
    void debug_draw(Scribe* scribe);

    void set_agent_target(float x, float y);
    void spawn_agents_circular(Vector2 center_vec, int agent_count);

    AgentManager();
    ~AgentManager();
};