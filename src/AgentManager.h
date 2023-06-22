#pragma once

#include <vector>
#include "Vector2.h"
#include "scribe.h"
#include "circlepacker.h"
#include "RVO.h"
#include "Agent.h"

#ifndef PIE
#define PIE 3.14159265358979323846f
#endif

#define BASE_RADIUS 5.f
#define NEAR_RADIUS BASE_RADIUS*2.f
#define BASE_RADIUSSQ BASE_RADIUS*BASE_RADIUS
#define NEAR_RADIUSSQ NEAR_RADIUS*NEAR_RADIUS

#define AT 2
#define NEAR 1
#define FAR 0

struct AgentGoal {
    int agentId;
    float x;
    float y;
    float radius; // for visualisation (can be got from agent vector)
};

struct Waypoint {
    Vector2 pos;
    //float x;
    //float y;
    float radius;
    float radius_min;
    float radius_max;
    float radius_growth;
};

struct DebugObstacle {
    std::vector<Vector2> poly;
};

class AgentGroup {
private:
    std::vector<RVO::Agent*> _agents;
    std::vector<Vector2> _route;
    int near_goal = 0;
    int at_goal = 0;
    bool deleteme = false;
public:
    AgentGroup() {};
    ~AgentGroup() {};
    friend class AgentManager;
};

class AgentManager {
private:
	std::vector<Waypoint*> _path;
    RVO::RVOSimulator* _rvoSim;
    CirclePacker* _circlePacker;

    /*
    * agents that have received a movement order will be put in a plan-group
    * agents that have been selected are in the _selected_agents vector.
    */
    // these agents have been selected by the player
    std::vector<RVO::Agent*> _selected_agents;
    std::vector<AgentGroup*> _agentgroups;
    std::vector<DebugObstacle*> _debugObstacles;
    std::vector<Vector2>* _temp_obstacle = nullptr;
    bool _clear_temp_obstacle = true;
    //bool _draw_debug_obstacles = true;
    size_t _group_count;
    bool _additive_selection;

    // for debug draw 
    bool _selection_changed;

    void update_rvof_velocities();
    void update_agent_groups(float deltaTime);
    void reset_selection();
    void set_agent_goals(const std::vector<RVO::Agent*>& agents, Vector2 goalpos);
    void stop_agents(const std::vector<RVO::Agent*>& agents);
    void set_next_goal_or_stop(AgentGroup* agentGroup);
    void calc_corner_offset(const Vector2& startpos, const std::vector<Waypoint*>& path);

    // debug draw flags
    bool renderKdTree = false;

    bool is_near(Vector2 distvec) const { return absSq(distvec) < NEAR_RADIUSSQ; }
    bool is_at(Vector2 distvec) const { return absSq(distvec) < BASE_RADIUSSQ; }

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

    void new_obstacle_vert(const Vector2& vec);
    void create_obstacle_from_verts();
    void create_new_obstacle(const std::vector<Vector2>& poly);

    void select_agent_point(float x, float y);
    void select_agent_box(float x0, float y0, float x1, float y1);

    // demo / debug functionality
    void spawn_agents_circular(Vector2 center_vec, int agent_count);
    void spawn_agents_square(Vector2 center_vec, int agent_count);
    void debug_draw(Scribe* scribe);

    AgentManager();
    ~AgentManager();
};