#include "AgentManager.h"

#ifndef PIE
#define PIE 3.14159265358979323846f
#endif

void AgentManager::init() {
    _rvoSim = new RVO::RVOSimulator();
    _circlePacker = new CirclePacker();
    _additive_selection = false;
}

void AgentManager::spawn_agents_circular(Vector2 center_vec, int agent_count) {
    float neighbourDist = 15.f;
    size_t maxNeighbours = 10;
    float timeHorizon = 10.f;
    float timeHorizonObst = 10.f;
    float radius = 5.f;
    float flocking_radius_scale = 5.f;
    float maxSpeed = 40.f;

    _rvoSim->setAgentDefaults(
        neighbourDist,
        maxNeighbours,
        timeHorizon,
        timeHorizonObst,
        radius,
        flocking_radius_scale,
        maxSpeed
    );

    float agent_distance = 250.f;

    for (int i = 0; i < agent_count; i++) {
        float ax = std::cos(i * 2.f * PIE / agent_count);
        float ay = std::sin(i * 2.f * PIE / agent_count);
        Vector2 relative_pos = Vector2(ax, ay) * agent_distance;
        Vector2 relative_goal = -relative_pos;
        spawn_agent(center_vec + relative_pos, center_vec + relative_goal);
    }
}

void AgentManager::spawn_agents_square(Vector2 center_vec, int agent_count) {
    float neighbourDist = 15.f;
    size_t maxNeighbours = 10;
    float timeHorizon = 10.f;
    float timeHorizonObst = 10.f;
    float radius = 5.f;
    float flocking_radius_scale = 5.f;
    float maxSpeed = 40.f;

    _rvoSim->setAgentDefaults(
        neighbourDist,
        maxNeighbours,
        timeHorizon,
        timeHorizonObst,
        radius,
        flocking_radius_scale,
        maxSpeed
    );

    Vector2 avgpos;
    float acs = std::sqrt(agent_count);
    int square_width = std::ceil(acs);
    int square_depth = std::floor(acs);
    int agents_placed = 0;
    std::vector<Vector2> place_positions;
    place_positions.resize(agent_count);

    for (int i = 0; i < square_depth; i++) {
        // check if remaining agents to be placed is smaller than square_depth
        int actual_width = std::min(square_width, agent_count - agents_placed);
        for (int j = 0; j < actual_width; j++) {
            // place agent
            Vector2 placepos = Vector2(j * neighbourDist, i * neighbourDist) + center_vec;
            spawn_agent(placepos);
        }
    }
}

void AgentManager::spawn_agent(Vector2 position, Vector2 goal, bool selected) {
    if (goal == Vector2()) {
        goal = position;
    }
    // make sure to keep these structures synced.
    _rvoSim->addAgent(position);
    _agent_goals.push_back(goal);
    //_selected_agents.push_back(std::pair<bool, int>(selected, _agent_goals.size()-1));
}

void AgentManager::set_additive_selection(bool active) {
    _additive_selection = active;
}

void AgentManager::set_selected_agent_targets(float x, float y) {
    // figure out which agents to move
    int agent_count = _selected_agents.size();

    // set up the circle packer
    _circlePacker->clear();
    _circlePacker->set_center(x,y);

    // setup and pack individual targets
    // incidentally, sync _circlePacker with agents vector
    for (int i = 0; i < agent_count; i++) {
        _circlePacker->add_circle(_selected_agents[i]->radius_);
    }
    _circlePacker->pack();

    // assign targets to agents
    for (int i = 0; i < agent_count; i++) {
        _agent_goals[_selected_agents[i]->id_] = _circlePacker->get_circle_position(i);
    }
}

void AgentManager::stop_selected_agents() {
    for (int i = 0; i < _selected_agents.size(); i++) {
        _agent_goals[_selected_agents[i]->id_] = _rvoSim->getAgentPosition(_selected_agents[i]->id_);
    }
}

void AgentManager::stop_all_agents() {
    int agent_count = _rvoSim->getNumAgents();
    _agent_goals.resize(agent_count);
    for (int i = 0; i < agent_count; i++) {
        _agent_goals[i] = _rvoSim->getAgentPosition(i);
    }
}

void AgentManager::new_waypoint(float x, float y) {
    Waypoint* newWaypoint = new Waypoint();

    newWaypoint->x = x;
    newWaypoint->y = y;
    newWaypoint->radius = 10.f;
    newWaypoint->radius_growth = -.3f;
    newWaypoint->radius_min = 5.f;
    newWaypoint->radius_max = 10.f;

    _path.push_back(newWaypoint);
}

bool AgentManager::rvof_reached_goals() {
    if (_rvoSim->getNumAgents() != _agent_goals.size()) {
        // we're out of sync
        return true;
    }

    for (int i = 0; i < _rvoSim->getNumAgents(); i++) {
        // check if the distance to the goal is less than the radius of our agent
        float radius = _rvoSim->getAgentRadius(i);
        if (absSq(_rvoSim->getAgentPosition(i) - _agent_goals[i]) > radius * radius) {
            return false;
        }
    }
    return true;
}

void AgentManager::update_rvof_velocities() {
    for (int i = 0; i < _rvoSim->getNumAgents(); i++) {
        Vector2 goalVector = _agent_goals[i] - _rvoSim->getAgentPosition(i);
        float max_speed = _rvoSim->getAgentMaxSpeed(i);

        goalVector = normalize(goalVector) * max_speed;

        _rvoSim->setAgentPrefVelocity(i, goalVector);
    }
}

void AgentManager::update(float deltaTime) {
    // do the RVO thing
    if (!rvof_reached_goals()) {
        _rvoSim->setTimeStep(deltaTime);
        update_rvof_velocities();
        _rvoSim->doStep();
    }

    for (int i = 0; i < _path.size(); i++) {
        _path[i]->radius += _path[i]->radius_growth;
        if (_path[i]->radius > _path[i]->radius_max || _path[i]->radius < _path[i]->radius_min)
            _path[i]->radius_growth *= -1.f;
    }
}

void AgentManager::reset_selection() {
    for (int i = 0; i < _selected_agents.size(); i++) {
        _selected_agents[i]->debug_draw_color_ = Color::YELLOW;
    }
    _selected_agents.clear();
}

void AgentManager::select_agent_point(float x, float y) {
    if (!_additive_selection) {
        reset_selection();
    }

    RVO::Agent* agent = _rvoSim->getAgentInPoint(Vector2(x, y));
    if (agent != nullptr) {
        // set the current selection to this agent
        // and also draw the selected agents in a different color
        agent->debug_draw_color_ = Color::GREEN;
        _selected_agents.push_back(agent);
    }
}

void AgentManager::select_agent_box(float x0, float y0, float x1, float y1) {
    if (!_additive_selection) {
        reset_selection();
    }
    
    Vector2 topleft = Vector2(std::min(x0, x1), std::min(y0, y1));
    Vector2 bottomright = Vector2(std::max(x0, x1), std::max(y0, y1));

    _rvoSim->getAgentsInRectangle(topleft, bottomright, &_selected_agents);
    for (int i = 0; i < _selected_agents.size(); i++) {
        _selected_agents[i]->debug_draw_color_ = Color::GREEN;
    }
}

void AgentManager::debug_draw(Scribe* scribe) {
    // draw KD tree
    if (renderKdTree) {
        _rvoSim->drawKdTree(scribe);
    }

    // draw the goals the agents are trying to reach
    scribe->set_draw_color(Color::WHITE);
    for (int i = 0; i < _agent_goals.size(); i++) {
        scribe->draw_circle(_agent_goals[i].x(), _agent_goals[i].y(), _rvoSim->getAgentRadius(i));
    }

    // Draw RVO agents
    std::vector<RVO::Agent*>* agentvec = _rvoSim->getAgentVector();

    for (int i = 0; i < _rvoSim->getNumAgents(); i++) {
        Vector2 p = _rvoSim->getAgentPosition(i);
        float r0 = _rvoSim->getAgentRadius(i);
        scribe->set_draw_color(Color(_rvoSim->getAgentDebugDrawColor(i)));
        scribe->draw_circle(p.x(), p.y(), std::ceil(r0));
    }

    // Draw Waypoints
    scribe->set_draw_color(Color::CYAN);
    for (int i = 0; i < _path.size(); i++) {
        scribe->draw_circle(_path[i]->x, _path[i]->y, _path[i]->radius);
        if (i + 1 != _path.size()) {
            scribe->draw_line(_path[i]->x, _path[i]->y, _path[i + 1]->x, _path[i + 1]->y);
        }
    }
}

void AgentManager::clear_waypoints() {
    for (int i = 0; i < _path.size(); i++) {
        delete _path[i];
    }
    _path.clear();
}

AgentManager::AgentManager() {
    init();
}

AgentManager::~AgentManager() {
    clear_waypoints();
    delete _rvoSim;
    delete _circlePacker;
}