#include "AgentManager.h"

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
    float radius = BASE_RADIUS;
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
    float radius = BASE_RADIUS;
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
    float acs = (float)std::sqrt(agent_count);
    int square_width = (int)std::ceil(acs);
    int square_depth = (int)std::floor(acs);
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
    int initialAgentState = MOVING;
    if (goal == Vector2()) {
        initialAgentState = IDLE;
    }
    size_t agentId = _rvoSim->addAgent(position);
    _rvoSim->setAgentGoal(agentId, goal);
    _rvoSim->setAgentState(agentId, initialAgentState);
}


void AgentManager::new_obstacle_vert(const Vector2& vec) {
    if (_clear_temp_obstacle) {
        _temp_obstacle = new std::vector<Vector2>();
        _clear_temp_obstacle = false;
    }
    _temp_obstacle->push_back(vec);
}

void AgentManager::create_obstacle_from_verts() {
    if (_temp_obstacle->size() > 2) {
        create_new_obstacle(*_temp_obstacle);
        _clear_temp_obstacle = true;
    }
}

void AgentManager::create_new_obstacle(const std::vector<Vector2>& poly) {
    DebugObstacle* dbo = new DebugObstacle{
        poly
    };
    _rvoSim->addObstacle(poly);
    _rvoSim->processObstacles();
    _debugObstacles.push_back(dbo);
}


void AgentManager::set_additive_selection(bool active) {
    _additive_selection = active;
}

void AgentManager::set_agent_goals(const std::vector<RVO::Agent*>& agents, Vector2 goalpos) {
    // set up the circle packer
    _circlePacker->clear();
    _circlePacker->set_center(goalpos);

    // setup and pack individual targets
    // incidentally, sync _circlePacker with agents vector
    for (int i = 0; i < agents.size(); i++) {
        _circlePacker->add_circle(agents[i]->radius_);
    }
    _circlePacker->pack();

    // assign targets to agents
    for (int i = 0; i < agents.size(); i++) {
        _rvoSim->setAgentGoal(agents[i]->id_, _circlePacker->get_circle_position(i));
        _rvoSim->setAgentState(agents[i]->id_, MOVING);
    }
}

// Calculate the offset we need on vec_b so that large groups can round the corners more easily
// return the offset vector for now TODO: in the future, we want to manipulate vec_b directly
void AgentManager::calc_corner_offset(const Vector2& avgAgentPos, const std::vector<Waypoint*>& path) {

    // make a copy of the path
    std::vector<Vector2> newpath;
    for (int i = 0; i < path.size(); i++) {
        newpath.push_back(Vector2(path[i]->pos));
    }

    // set up a walk through all points in path
    Vector2 vec_a = avgAgentPos;
    Vector2 vec_b = path[0]->pos;
    Vector2 vec_c = path[1]->pos;

    // starting from after the setup; calculate the offset for our path nodes
    for (int i = 2; i < path.size(); i++) {

    }
}

void AgentManager::set_selected_agent_targets(float x, float y) {
    // generate path from pathfinding algo
    // shortest path is a std::vector<Vector2> of size 1
    // for debugging purposes check if we can use the waypoints.
    AgentGroup* agentGroup = new AgentGroup();
    agentGroup->near_goal = 0;
    agentGroup->at_goal = 0;
    Vector2 avgPos = Vector2();
    for (int i = 0; i < _selected_agents.size(); i++) {
        agentGroup->_agents.push_back(_selected_agents[i]);
        avgPos += _selected_agents[i]->position_;
    }
    avgPos /= _selected_agents.size();

    // if _path.size >= 1 (and we've right-clicked near the starting circle of the path) then we've 
    // received a complex path from the pathfinder and we first will need to adjust it so it will 
    // be easier for the selected agents to round any corners.
    if (_path.size() >= 1) {
        if (absSq(Vector2(x, y) - _path[0]->pos) < 100.f) {

            // first adjust the _path
            calc_corner_offset(avgPos, _path);

            for (int i = 0; i < _path.size(); i++) {
                agentGroup->_route.push_back(_path[i]->pos);
            }
        }
    }
    else {
        //new_waypoint(x, y);
        agentGroup->_route.push_back(Vector2(x, y));
    }

    // figure out which agents to move
    int agent_count = _selected_agents.size();

    // get average x,y
    // and also assign all selected agents to the same move-group
    Vector2 avgpos = Vector2();
    for (int i = 0; i < agent_count; i++) {
        _selected_agents[i]->agent_group_ = agentGroup;
        avgpos += _selected_agents[i]->position_;
    }
    avgpos /= agent_count;

    set_agent_goals(agentGroup->_agents, Vector2(x, y));

    _agentgroups.push_back(agentGroup);

    // we must trigger this for the debug draw
    _selection_changed = true;
}

void AgentManager::stop_selected_agents() {
    for (int i = 0; i < _selected_agents.size(); i++) {
        _rvoSim->setAgentGoal(_selected_agents[i]->id_, _rvoSim->getAgentPosition(_selected_agents[i]->id_));
        _rvoSim->setAgentState(_selected_agents[i]->id_, IDLE);
    }
}

void AgentManager::stop_agents(const std::vector<RVO::Agent*>& agents) {
    for (int i = 0; i < agents.size(); i++) {
        _rvoSim->setAgentGoal(agents[i]->id_, _rvoSim->getAgentPosition(agents[i]->id_));
        _rvoSim->setAgentState(agents[i]->id_, IDLE);
    }
}

void AgentManager::stop_all_agents() {
    int agent_count = _rvoSim->getNumAgents();
    for (int i = 0; i < agent_count; i++) {
        _rvoSim->setAgentGoal(i, _rvoSim->getAgentPosition(i));
        _rvoSim->setAgentState(i, IDLE);
    }
}

void AgentManager::new_waypoint(float x, float y) {
    Waypoint* newWaypoint = new Waypoint();

    newWaypoint->pos = Vector2(x, y);
    newWaypoint->radius = 10.f;
    newWaypoint->radius_growth = -.3f;
    newWaypoint->radius_min = 5.f;
    newWaypoint->radius_max = 10.f;

    _path.push_back(newWaypoint);
}

void AgentManager::update_rvof_velocities() {
    for (int i = 0; i < _rvoSim->getNumAgents(); i++) {
        Vector2 goalVector;

        if (_rvoSim->getAgentState(i) == MOVING) {
            goalVector = _rvoSim->getAgentGoal(i) - _rvoSim->getAgentPosition(i);
            float max_speed = _rvoSim->getAgentMaxSpeed(i);
            goalVector = normalize(goalVector) * max_speed;
        }

        _rvoSim->setAgentPrefVelocity(i, goalVector);
    }
}

void AgentManager::set_next_goal_or_stop(AgentGroup* agentGroup) {
    // if there are no more next_points -> stop the group
    if (agentGroup->_route.size() <= 1) {
        stop_agents(agentGroup->_agents);
        agentGroup->deleteme = true;
    }
    else {
        agentGroup->_route.pop_back();

        // reset the agentgroup
        agentGroup->near_goal = 0;
        agentGroup->at_goal = 0;

        // set new goals for the agents
        set_agent_goals(agentGroup->_agents, agentGroup->_route[agentGroup->_route.size()-1]);
    }
}

void AgentManager::update_agent_groups(float deltaTime) {
    bool stopmoving = false;
    for (int i = 0; i < _agentgroups.size(); i++) {
        Vector2 avgAgentPos;
        Vector2 avgGoalPos;
        for (int j = 0; j < _agentgroups[i]->_agents.size(); j++) {
            avgAgentPos += _agentgroups[i]->_agents[j]->position_;
            avgGoalPos += _agentgroups[i]->_agents[j]->goal_;
        }
        avgAgentPos /= _agentgroups[i]->_agents.size();
        avgGoalPos /= _agentgroups[i]->_agents.size();

        if (is_at(avgAgentPos - avgGoalPos)) {
            set_next_goal_or_stop(_agentgroups[i]);
        }
    }

    for (int i = 0; i < _agentgroups.size(); i++) {
        if (_agentgroups[i]->deleteme) {
            _agentgroups[i]->_agents.clear();
            _agentgroups.erase(_agentgroups.begin() + i);
            i--;
        }
    }
}

void AgentManager::update(float deltaTime) {
    _rvoSim->setTimeStep(deltaTime);
    update_rvof_velocities();
    _rvoSim->doStep();

    // update agent_groups
    update_agent_groups(deltaTime);

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
    _selection_changed = true;
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
    _selection_changed = true;
}

void AgentManager::debug_draw(Scribe* scribe) {
    if (renderKdTree) {
        _rvoSim->drawKdTree(scribe);
    }

    // draw goals
    scribe->set_draw_color(Color::WHITE);
    for (int i = 0; i < _rvoSim->getNumAgents(); i++) {
        Vector2 agentGoal = _rvoSim->getAgentGoal(i);
        scribe->draw_circle(agentGoal.x(), agentGoal.y(), _rvoSim->getAgentRadius(i));
    }

    // draw agents
    for (int i = 0; i < _rvoSim->getNumAgents(); i++) {
        Vector2 p = _rvoSim->getAgentPosition(i);
        float r0 = _rvoSim->getAgentRadius(i);
        scribe->set_draw_color(Color(_rvoSim->getAgentDebugDrawColor(i)));
        scribe->draw_circle(p.x(), p.y(), std::ceil(r0));
    }

    bool draw_offset_vec = false;
    // draw Waypoints /w offsets
    for (int i = 0; i < _path.size(); i++) {
        scribe->set_draw_color(Color::CYAN);
        scribe->draw_circle(_path[i]->pos.x(), _path[i]->pos.y(), _path[i]->radius);
        if (i + 1 != _path.size()) {
            scribe->set_draw_color(Color::CYAN);
            scribe->draw_line(_path[i]->pos.x(), _path[i]->pos.y(), _path[i + 1]->pos.x(), _path[i + 1]->pos.y());
            draw_offset_vec = true;
        }
    }

    // draw _temp_obstacle
    if (_temp_obstacle != nullptr) {
        for (int i = 0; i < _temp_obstacle->size(); i++) {
            scribe->set_draw_color(Color::WHITE);
            scribe->draw_circle(_temp_obstacle->at(i).x(), _temp_obstacle->at(i).y(), 3.f);
            if (i + 1 < _temp_obstacle->size()) {
                scribe->draw_line(_temp_obstacle->at(i).x(), _temp_obstacle->at(i).y(), _temp_obstacle->at(i + 1).x(), _temp_obstacle->at(i + 1).y());
            }
        }
        if (_temp_obstacle->size() > 2) {
            scribe->draw_line(_temp_obstacle->at(0).x(), _temp_obstacle->at(0).y(), _temp_obstacle->back().x(), _temp_obstacle->back().y());
        }
    }

    // draw obstacles
    for (int i = 0; i < _debugObstacles.size(); i++) {
        scribe->set_draw_color(Color::YELLOW);
        for (int j = 0; j < _debugObstacles[i]->poly.size(); j++) {
            if (j + 1 < _debugObstacles[i]->poly.size()) {
                scribe->draw_line(_debugObstacles[i]->poly[j].x(), _debugObstacles[i]->poly[j].y(), _debugObstacles[i]->poly[j+1].x(), _debugObstacles[i]->poly[j+1].y());
            }
            else {
                scribe->draw_line(_debugObstacles[i]->poly[j].x(), _debugObstacles[i]->poly[j].y(), _debugObstacles[i]->poly[0].x(), _debugObstacles[i]->poly[0].y());
            }
        }
    }

    // draw offset vec on the path
    if (_selected_agents.size() > 0 && draw_offset_vec) {

        if (_selection_changed) {
            _circlePacker->clear();
            _circlePacker->set_center(Vector2());

            // setup and pack individual targets
            // incidentally, sync _circlePacker with agents vector
            for (int i = 0; i < _selected_agents.size(); i++) {
                _circlePacker->add_circle(_selected_agents[i]->radius_);
            }
            _circlePacker->pack();
            _selection_changed = false;
        }

        Vector2 avgpos = Vector2();
        for (int i = 0; i < _selected_agents.size(); i++) {
            //_selected_agents[i]->agent_group_ = agentGroup;
            avgpos += _selected_agents[i]->position_;
        }
        avgpos /= _selected_agents.size();

        Vector2 path0vec = _path[0]->pos;
        Vector2 path1vec = _path[1]->pos;
        Vector2 avgpos_to_path0vec = _path[0]->pos - avgpos;
        Vector2 path0_to_path1vec = _path[1]->pos - _path[0]->pos;
        Vector2 path1_to_avgposvec = avgpos - _path[1]->pos;

        scribe->set_draw_color(Color::BLUE);
        scribe->draw_line(avgpos.x(), avgpos.y(), path0vec.x(), path0vec.y());

        // the corner is not as sharp and we can offset the target a bit
        if (dot(avgpos_to_path0vec, path0_to_path1vec) > 0.f) {
            // now we want to get the normal of path0-to-path1-vec (hopefully it has the correct sign, otherwise change sign)
            // place it on the tip of avgpos-to-path0 vec 
            // and have its magnitude be big enough so that its radius is at least equal to the radius of the selected units combined

            Vector2 checkvec = rotate90(path0_to_path1vec);
            Vector2 offsetvec = normalize(avgpos_to_path0vec);
            // avgpos_to_path0vec lies to the right of path0_to_path1vec thus we must rotate90 counter-clockwise
            if (dot(checkvec, avgpos_to_path0vec) > 0.f) {
                offsetvec = rotate90(offsetvec, COUNTERCLOCKWISE);
            }
            // avgpos_to_path0vec lies to the left of path0_to_path1vec thus we must rotate90 clockwise
            else {
                offsetvec = rotate90(offsetvec, CLOCKWISE);
            }
            offsetvec *= _circlePacker->get_approx_diameter() / 2.f;
            offsetvec += path0vec;
            scribe->set_draw_color(Color::RED);
            scribe->draw_line(path0vec.x(), path0vec.y(), offsetvec.x(), offsetvec.y());
        }
        else {  // the corner is sharper than 90deg; we must offset
            // now we want to get the center of the triangle formed by avgpos-to-path0, path0-to-path1 & path1-to-avgpos
            // from there we make a new vec => centervec-to-path0
            // and add another centervec-to-path0 vec with a magnitude that is at least equal to the radius of the selected units combined
            Vector2 checkvec = rotate90(path0_to_path1vec);
            Vector2 offsetvec = normalize(avgpos_to_path0vec);
            // avgpos_to_path0vec lies to the right of path0_to_path1vec thus we must rotate90 counter-clockwise
            if (dot(checkvec, avgpos_to_path0vec) > 0.f) {
                offsetvec = rotate90(offsetvec, COUNTERCLOCKWISE);
            }
            // avgpos_to_path0vec lies to the left of path0_to_path1vec thus we must rotate90 clockwise
            else {
                offsetvec = rotate90(offsetvec, CLOCKWISE);
            }
            offsetvec *= _circlePacker->get_approx_diameter() / 2.f;
            offsetvec += path0vec;
            scribe->set_draw_color(Color::RED);
            scribe->draw_line(path0vec.x(), path0vec.y(), offsetvec.x(), offsetvec.y());
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
    _debugObstacles.clear();
}