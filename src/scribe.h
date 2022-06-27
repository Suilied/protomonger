#pragma once

#include <filesystem>
#include <math.h>
#include <vector>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#define DEFAULT_FONT ".\\assets\\fonts\\Hack-Regular.ttf"

enum Color {
    BLACK,
    WHITE,
    RED,
    YELLOW,
    GREEN,
    CYAN,
    BLUE,
    NONE
};

class Scribe {
private: 
    std::vector<SDL_Color> _colors;
    SDL_Renderer* _renderer;
    Color _draw_color;
    Color _clear_color;
    TTF_Font* _font;
    SDL_Surface* _msg_sur;
    SDL_Texture* _msg_tex;

    void draw_quadrant(SDL_Renderer* renderer, int a, int b, int x, int y);

public:
    void set_renderer(SDL_Renderer* renderer);
    void set_draw_color(Color color);
    void set_clear_color(Color color);
    void draw_circle(int x, int y, int radius);
    void draw_line(int x0, int y0, int x1, int y1);
    void draw_rectangle(int x0, int y0, int x1, int y1);
    void draw_text(int x, int y, const char* t);
    void clear();
    void draw();
    Scribe(SDL_Renderer* renderer);
    ~Scribe();
};