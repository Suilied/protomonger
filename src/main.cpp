#define SDL_MAIN_HANDLED

#include <iostream>
#include <string>
#include <SDL2/SDL.h>
#include <random>

#include "scribe.h"
#include "circlepacker.h"
#include "RVO.h"

#define PIE 3.14159265358979323846f

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define WINDOW_CENTERX WINDOW_WIDTH/2
#define WINDOW_CENTERY WINDOW_HEIGHT/2

#define NUM_AGENTS 30

struct Bubble {
    int pack_id;
    float x;
    float y;
    float radius;
};

std::vector<Bubble*> bubbles;

void clear_bubbles() {
    for (int i = 0; i < bubbles.size(); i++) {
        delete bubbles[i];
    }
    bubbles.clear();
}

void prep_bubbles(CirclePacker* circlePacker, int amount) {
    std::random_device rd;
    std::mt19937 rand_generator(rd());
    std::uniform_real_distribution<> rand_radius_distribution(0.f, 15.f);

    float min_radius = 10.f;

    for (int i = 0; i < amount; i++) {
        Bubble* a = new Bubble();
        a->pack_id = i;
        a->x = 0.f;
        a->y = 0.f;
        a->radius = min_radius + rand_radius_distribution(rand_generator);
        bubbles.push_back(a);

        circlePacker->add_circle(a->radius);
    }

    circlePacker->set_center(WINDOW_CENTERX, WINDOW_CENTERY);
    circlePacker->pack();
}

void set_agent_targets(CirclePacker* circlePacker, RVO::RVOSimulator* rvoSim, std::vector<Vector2>* agent_goals) {
    // clear everything
    agent_goals->clear();
    circlePacker->clear();
    clear_bubbles();

    int agent_count = rvoSim->getNumAgents();

    // setup and pack individual targets
    for (int i = 0; i < agent_count; i++) {
        Bubble* a = new Bubble();
        a->pack_id = i;
        a->x = 0.f;
        a->y = 0.f;
        a->radius = rvoSim->getAgentRadius(i);
        bubbles.push_back(a);

        circlePacker->add_circle(a->radius);
    }
    circlePacker->pack();

    // assign targets to agents
    for (int i = 0; i < agent_count; i++) {
        Vector2 agent_goal = circlePacker->get_circle_position(i);
        agent_goals->push_back(agent_goal);
    }
}

void shuffle_bubbles(CirclePacker* circlePacker) {
    std::random_device rd;
    std::mt19937 rand_generator(rd());
    std::uniform_real_distribution<> rand_radius_distribution(0.f, 25.f);

    float min_radius = 10.f;
    for (int i = 0; i < bubbles.size(); i++) {
        float new_radius = min_radius + rand_radius_distribution(rand_generator);
        bubbles[i]->radius = new_radius;
        circlePacker->set_circle_radius(i, new_radius);
    }

    circlePacker->pack();
}

void reroll_bubbles(CirclePacker* circlePacker) {
    circlePacker->clear();
    clear_bubbles();

    std::random_device rd;
    std::mt19937 rand_generator(rd());
    std::uniform_real_distribution<> rand_radius_distribution(0.f, 25.f);
    std::uniform_int_distribution<> rand_agentcount_distribution(8, 50);
    int agent_count = rand_agentcount_distribution(rand_generator);
    float min_radius = 10.f;

    for (int i = 0; i < agent_count; i++) {
        Bubble* a = new Bubble();
        a->pack_id = i;
        a->x = 0.f;
        a->y = 0.f;
        a->radius = min_radius + rand_radius_distribution(rand_generator);
        bubbles.push_back(a);

        circlePacker->add_circle(a->radius);
    }

    circlePacker->set_center(WINDOW_CENTERX, WINDOW_CENTERY);
    circlePacker->pack();
}

void prep_rvof(RVO::RVOSimulator* rvoSim, std::vector<Vector2>* agent_goals, int agent_count) {
    float neighbourDist = 15.f;
    size_t maxNeighbours = 10;
    float timeHorizon = 10.f;
    float timeHorizonObst = 10.f;
    float radius = 5.f;
    float flocking_radius_scale = 5.f;
    float maxSpeed = 40.f;

    rvoSim->setAgentDefaults(
        neighbourDist,
        maxNeighbours,
        timeHorizon,
        timeHorizonObst,
        radius,
        flocking_radius_scale,
        maxSpeed
    );

    float agent_distance = 250.f;
    Vector2 center_vec = Vector2(WINDOW_CENTERX, WINDOW_CENTERY);

    for (int i = 0; i < agent_count; i++) {
        float ax = std::cos(i * 2.f * PIE / agent_count);
        float ay = std::sin(i * 2.f * PIE / agent_count);
        Vector2 relative_pos = Vector2(ax, ay) * agent_distance;
        Vector2 relative_goal = -relative_pos;
        rvoSim->addAgent(center_vec + relative_pos);
        agent_goals->push_back(center_vec + relative_goal);
    }
}

void update_rvof_velocities(RVO::RVOSimulator* rvoSim, std::vector<Vector2>* agent_goals) {
    for (int i = 0; i < rvoSim->getNumAgents(); i++) {
        Vector2 goalVector = agent_goals->at(i) - rvoSim->getAgentPosition(i);
        float max_speed = rvoSim->getAgentMaxSpeed(i);

        goalVector = normalize(goalVector) * max_speed;

        rvoSim->setAgentPrefVelocity(i, goalVector);
    }
}

void update_bubble_positions(CirclePacker* circlePacker) {
    for (int i = 0; i < bubbles.size(); i++) {
        Vector2 new_pos = circlePacker->get_circle_position(bubbles[i]->pack_id);
        bubbles[i]->x = new_pos.x();
        bubbles[i]->y = new_pos.y();
    }
}

bool rvof_reached_goals(RVO::RVOSimulator* rvoSim, std::vector<Vector2>* agent_goals) {
    // TODO: move this functionality to the Agent propper
    // in addition add functionality that allows agents to propegate their "goal-reached (close enough)" state
    // then when all agents are also within an outer radius (+a bit of leeway; all calculated by the circlepacker)
    // tell all agents that they've sorta reached their goal and make them stop moving.

    if (rvoSim->getNumAgents() != agent_goals->size()) {
        // we're out of sync
        return true;
    }

    for (int i = 0; i < rvoSim->getNumAgents(); i++) {
        // check if the distance to the goal is less than the radius of our agent
        float radius = rvoSim->getAgentRadius(i);
        if (absSq(rvoSim->getAgentPosition(i) - agent_goals->at(i)) > radius * radius) {
            return false;
        }
    }
    return true;
}

int main(int argc, char* argv[])
{
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Surface* surface;
    SDL_Texture* texture;
    SDL_Event event;
    SDL_bool exit_program = SDL_FALSE;

    if(SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't init SDL: %s", SDL_GetError());
        return 3;
    }

    int windowFlags = 0;
    window = SDL_CreateWindow("asdasd", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, windowFlags);
    if(!window) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't create window: %s", SDL_GetError());
        return 3;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if(!renderer) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't create renderer: %s", SDL_GetError());
        return 3;
    }

    Scribe* scribe = new Scribe(renderer);
    CirclePacker* circlePacker = new CirclePacker();
    RVO::RVOSimulator* rvoSim = new RVO::RVOSimulator();
    std::vector<Vector2> agent_goals;

    prep_rvof(rvoSim, &agent_goals, NUM_AGENTS);

    // prep our circles for packing
    int bubble_amount = 18;
    prep_bubbles(circlePacker, bubble_amount);

    // setup frame timer
    unsigned int total_time = SDL_GetTicks();
    unsigned int frame_time = 0;

    while(!exit_program) {
        // first, get the time between frames
        unsigned int new_time = SDL_GetTicks();
        frame_time = new_time - total_time;
        total_time = new_time;
        float frame_time_ms = frame_time / 1000.0f;

        while(SDL_PollEvent(&event)) {
            switch(event.type) {
                case SDL_QUIT:
                    exit_program = SDL_TRUE;
                    break;
                case SDL_KEYDOWN:
                    switch (event.key.keysym.sym) {
                        case SDLK_SPACE:
                            shuffle_bubbles(circlePacker);
                            break;
                        case SDLK_r:
                            reroll_bubbles(circlePacker);
                            break;
                        case SDLK_ESCAPE:
                            exit_program = SDL_TRUE;
                            break;
                    }
                    break;
                case SDL_MOUSEBUTTONDOWN:
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        //circlePacker->set_center(event.button.x, event.button.y);
                    }
                    if (event.button.button == SDL_BUTTON_RIGHT) {
                        circlePacker->set_center(event.button.x, event.button.y);
                        set_agent_targets(circlePacker, rvoSim, &agent_goals);
                    }
                    break;
            }
        }

        scribe->clear();

        // do the RVO thing
        if (!rvof_reached_goals(rvoSim, &agent_goals)) {
            rvoSim->setTimeStep(frame_time_ms);
            update_rvof_velocities(rvoSim, &agent_goals);
            rvoSim->doStep();
        }

        // Draw RVO agents
        for (int i = 0; i < rvoSim->getNumAgents(); i++) {
            Vector2 p = rvoSim->getAgentPosition(i);
            float r0 = rvoSim->getAgentRadius(i);
            //float r1 = rvoSim->getAgentFlockingRadius(i);
            scribe->set_draw_color(Color::YELLOW);
            scribe->draw_circle(p.x(), p.y(), std::ceil(r0));
            //scribe->set_draw_color(Color::BLUE);
            //scribe->draw_circle(p.x(), p.y(), std::ceil(r1));
        }

        // draw the actual goals the agents are trying to reach
        scribe->set_draw_color(Color::WHITE);
        for (int i = 0; i < agent_goals.size(); i++) {
            //rvoSim->getagentgpos
            scribe->draw_circle(agent_goals[i].x(), agent_goals[i].y(), rvoSim->getAgentRadius(i));
        }

        int fps_val = std::floor(frame_time > 0 ? 1000.f / frame_time : 0.f);
        std::string fps_text = std::to_string(fps_val);
        scribe->draw_text(10, 10, fps_text.c_str());

        scribe->draw();
    }

    SDL_Quit();
    return 0;
}