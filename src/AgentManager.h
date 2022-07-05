#pragma once

#include <vector>
#include "Vector2.h"
#include "scribe.h"
#include "circlepacker.h"
#include "RVO.h"
#include "Agent.h"

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

    /*
    * agents that have received a movement order will be put in a plan-group
    * agents that have been selected are in the _selected_agents vector.
    */
    // these agents have been selected by the player
    std::vector<RVO::Agent*> _selected_agents;
    size_t _group_count;
    bool _additive_selection;

    bool rvof_reached_goals();
    void update_rvof_velocities();
    void reset_selection();

    // debug draw flags
    bool renderKdTree = false;

public:
    void init();
    void update(float deltaTime);
	void new_waypoint(float x, float y);
    void clear_waypoints();

    void set_additive_selection(bool active);
    void set_selected_agent_targets(float x, float y);
    void stop_selected_agents();
    void stop_all_agents();
    void spawn_agent(Vector2 position, Vector2 goal = Vector2(), bool selected = false);

    void select_agent_point(float x, float y);
    void select_agent_box(float x0, float y0, float x1, float y1);

    // demo / debug functionality
    void spawn_agents_circular(Vector2 center_vec, int agent_count);
    void spawn_agents_square(Vector2 center_vec, int agent_count);
    void debug_draw(Scribe* scribe);

    AgentManager();
    ~AgentManager();
};