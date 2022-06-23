#define SDL_MAIN_HANDLED

#include <iostream>
#include <string>
#include <SDL2/SDL.h>
#include <random>

#include "scribe.h"
#include "circlepacker.h"
#include "AgentManager.h"
#include "RVO.h"

/*
* tour agency nodig
* tour guides & planners
* in agent een stuk met status (enum) 
* zodat we andere agents kunnen pushen als die idle zijn
* planner heeft circlepacker nodig om sub-stations correct te plannen
* substations krijgen een offset in hoeken everredig aan de hoek
* offset lengte van de hoek is totaal size van circlepacker + constante
* los van de tour agency moet er een manier zijn om agents te selecteren
*/

#ifndef PIE
#define PIE 3.14159265358979323846f
#endif

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define WINDOW_CENTERX WINDOW_WIDTH/2
#define WINDOW_CENTERY WINDOW_HEIGHT/2

#define NUM_AGENTS 30

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

    AgentManager* agentPlanner = new AgentManager();
    agentPlanner->spawn_agents_circular(Vector2(WINDOW_CENTERX, WINDOW_CENTERY),NUM_AGENTS);

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
                            break;
                        case SDLK_r:
                            break;
                        case SDLK_ESCAPE:
                            exit_program = SDL_TRUE;
                            break;
                    }
                    break;
                case SDL_MOUSEBUTTONDOWN:
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        // add point to path planner
                        agentPlanner->new_waypoint(event.button.x, event.button.y);
                    }
                    if (event.button.button == SDL_BUTTON_MIDDLE) {
                        // erase constructed path
                        agentPlanner->clear_waypoints();
                    }
                    if (event.button.button == SDL_BUTTON_RIGHT) {
                        agentPlanner->set_agent_target(event.button.x, event.button.y);
                    }
                    break;
            }
        }

        // Clear the screen / set background color
        scribe->clear();

        // do all agent related stuff
        agentPlanner->update(frame_time_ms);
        agentPlanner->debug_draw(scribe);

        // draw FPS
        int fps_val = std::floor(frame_time > 0 ? 1000.f / frame_time : 0.f);
        std::string fps_text = std::to_string(fps_val);
        scribe->draw_text(10, 10, fps_text.c_str());

        scribe->draw();
    }

    SDL_Quit();
    return 0;
}