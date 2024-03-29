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

enum MouseMode {
    select = 0,
    waypoint,
    obstacle,
    max
};

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
    //agentPlanner->spawn_agents_circular(Vector2(WINDOW_CENTERX, WINDOW_CENTERY),NUM_AGENTS);
    agentPlanner->spawn_agents_square(Vector2(WINDOW_CENTERX, WINDOW_CENTERY), NUM_AGENTS);

    int mousemode = MouseMode::select;
    bool selecting = false;
    Vector2 select_box;
    Vector2 mouse_pos;

    // setup frame timer
    unsigned int total_time = SDL_GetTicks();
    unsigned int frame_time = 0;


    // TODO: 
    // make units follow waypoints to the end
    // when consuming a waypoint and deciding on the next:
    // - calculate offset for cornering
    // - using an approximate total-size calculation
    // implement unit state
    // implement "good enough" when trying to reach destination
    // - this is based on state + team
    // - solve for intermediate waypoints as well, 
    //   i.e. the units move as a whole
    // implement "pushing" that depends on state + team
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
                            // if building an obstacle, finish build and push it to the RVOSim
                            if (mousemode == MouseMode::obstacle) {
                                agentPlanner->create_obstacle_from_verts();
                            }
                            break;
                        case SDLK_x:
                            if (mousemode != MouseMode::max - 1)
                                mousemode++;
                            else
                                mousemode = 0;
                            break;
                        case SDLK_s:
                            // stop selected agents from moving
                            agentPlanner->stop_selected_agents();
                            break;
                        case SDLK_h:
                            // stop all agents from moving
                            agentPlanner->stop_all_agents();
                            break;
                        case SDLK_ESCAPE:
                            exit_program = SDL_TRUE;
                            break;
                        case SDLK_LSHIFT:
                        case SDLK_RSHIFT:
                            agentPlanner->set_additive_selection(true);
                            break;
                        case SDLK_LEFT:
                            scribe->add_move_camera(Vector2(100.f, 0.f));
                            break;
                        case SDLK_RIGHT:
                            scribe->add_move_camera(Vector2(-100.f, 0.f));
                            break;
                        case SDLK_UP:
                            scribe->add_move_camera(Vector2(0.f, 100.f));
                            break;
                        case SDLK_DOWN:
                            scribe->add_move_camera(Vector2(0.f, -100.f));
                            break;
                    }
                    break;
                case SDL_KEYUP:
                    switch (event.key.keysym.sym) {
                        case SDLK_LSHIFT:
                        case SDLK_RSHIFT:
                            agentPlanner->set_additive_selection(false);
                            break;
                    }
                    break;
                case SDL_MOUSEBUTTONDOWN:
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        // TODO:
                        // right click on waypoint will set that as the destination of the selected group.
                        switch (mousemode) {
                        case MouseMode::select:
                            select_box = scribe->get_mouse_vec();
                            selecting = true;
                            mouse_pos = select_box;
                            break;
                        case MouseMode::waypoint:
                            // add point to path planner
                            agentPlanner->new_waypoint(scribe->get_mouse_x(), scribe->get_mouse_y());
                            break;
                        case MouseMode::obstacle:
                            // add vector to obstacle
                            agentPlanner->new_obstacle_vert(scribe->get_mouse_vec());
                            break;
                        }
                    }
                    if (event.button.button == SDL_BUTTON_MIDDLE) {
                        // erase constructed path
                        agentPlanner->clear_waypoints();
                    }
                    if (event.button.button == SDL_BUTTON_RIGHT) {
                        agentPlanner->set_selected_agent_targets(scribe->get_mouse_x(), scribe->get_mouse_y());
                    }
                    break;
                case SDL_MOUSEBUTTONUP:
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        if (mousemode == MouseMode::select) {
                            // if area of selection box is > 5x5 px
                            float selboxSize = (scribe->get_mouse_x() - select_box.x()) * (scribe->get_mouse_y() - select_box.y());
                            selboxSize = selboxSize < 0.f ? selboxSize * -1.f : selboxSize;
                            if (selboxSize > 20.f && selecting) {
                                agentPlanner->select_agent_box(select_box.x(), select_box.y(), scribe->get_mouse_x(), scribe->get_mouse_y());
                            }
                            else {
                                agentPlanner->select_agent_point(scribe->get_mouse_x(), scribe->get_mouse_y());
                            }
                            selecting = false;
                            select_box = Vector2();
                        }
                    }
                    break;
                case SDL_MOUSEMOTION:
                    scribe->update_mouse(event.button.x, event.button.y);
                    mouse_pos = scribe->get_mouse_vec();
                    break;
            }
        }

        // Clear the screen / set background color
        scribe->clear();
        scribe->update(frame_time_ms);

        // do all agent related stuff
        agentPlanner->update(frame_time_ms);
        agentPlanner->debug_draw(scribe);

        // draw cam+mouse stuff for debug
        scribe->set_draw_color(Color::PURPLE);
        scribe->draw_circle(scribe->get_mouse_x(), scribe->get_mouse_y(), 2);

        // draw selection box (if applicable)
        if (selecting) {
            scribe->set_draw_color(Color::GREEN);
            scribe->draw_rectangle(select_box.x(), select_box.y(), mouse_pos.x(), mouse_pos.y());
        }

        // draw FPS
        int fps_val = std::floor(frame_time > 0 ? 1000.f / frame_time : 0.f);
        std::string fps_text = std::to_string(fps_val);
        scribe->draw_text(10, 10, fps_text.c_str());

        switch (mousemode) {
        case MouseMode::select:
            scribe->draw_text(10, 20, "Mode: Select");
            break;
        case MouseMode::waypoint:
            scribe->draw_text(10, 20, "Mode: Waypoint");
            break;
        case MouseMode::obstacle:
            scribe->draw_text(10, 20, "Mode: Obstacle");
        }

        scribe->draw();
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}