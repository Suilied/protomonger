#include "scribe.h"

void Scribe::draw_quadrant(SDL_Renderer* renderer, int a, int b, int x, int y) {
	SDL_RenderDrawPoint(renderer, a+x, b+y);
	SDL_RenderDrawPoint(renderer, -a+x, b+y);
	SDL_RenderDrawPoint(renderer, a+x, -b+y);
	SDL_RenderDrawPoint(renderer, -a+x, -b+y);
	SDL_RenderDrawPoint(renderer, b+x, a+y);
	SDL_RenderDrawPoint(renderer, b+x, -a+y);
	SDL_RenderDrawPoint(renderer, -b+x, a+y);
	SDL_RenderDrawPoint(renderer, -b+x, -a+y);
}

void Scribe::draw_circle(int x, int y, int radius) {
    SDL_SetRenderDrawColor(_renderer, _colors[_draw_color].r, _colors[_draw_color].g, _colors[_draw_color].b, _colors[_draw_color].a);

    Vector2 camvec = _camera->get_position();
    x += camvec.x();
    y += camvec.y();
    //radius *= _camera->get_zoom();
    //TODO: add zoom

    int pk = 3 - 2*radius;
    int a=0;
    int b=radius;
    draw_quadrant(_renderer, a, b, x, y);
    while(a < b) {
        if(pk <= 0) {
            pk = pk + (4*a) + 6;
            draw_quadrant(_renderer, ++a, b, x, y);
        }
        else {
            pk = pk + (4*(a-b))+10;
            draw_quadrant(_renderer, ++a, --b, x, y);
        }
    }
}

void Scribe::draw_line(int x0, int y0, int x1, int y1) {

    Vector2 camvec = _camera->get_position();
    x0 += camvec.x();
    x1 += camvec.x();
    y0 += camvec.y();
    y1 += camvec.y();
    //TODO: add zoom

    SDL_SetRenderDrawColor(_renderer, _colors[_draw_color].r, _colors[_draw_color].g, _colors[_draw_color].b, _colors[_draw_color].a);
    SDL_RenderDrawLine(_renderer, x0, y0, x1, y1);
}

void Scribe::draw_rectangle(int x0, int y0, int x1, int y1) {
    draw_line(x0, y0, x1, y0);
    draw_line(x1, y0, x1, y1);
    draw_line(x1, y1, x0, y1);
    draw_line(x0, y1, x0, y0);
}

void Scribe::draw_text(int x, int y, const char* t) {
    _msg_sur = TTF_RenderText_Solid(_font, t, _colors[Color::WHITE]);

    int w, h;
    TTF_SizeText(_font, t, &w, &h);
    _msg_tex = SDL_CreateTextureFromSurface(_renderer, _msg_sur);
    SDL_FreeSurface(_msg_sur);

    SDL_Rect msg_rect;
    msg_rect.x = x;
    msg_rect.y = y;
    msg_rect.w = w;
    msg_rect.h = h;

    SDL_RenderCopy(_renderer, _msg_tex, NULL, &msg_rect);
    SDL_DestroyTexture(_msg_tex);
}

void Scribe::set_renderer(SDL_Renderer* renderer) {
    _renderer = renderer;
}

void Scribe::set_draw_color(Color color) {
    _draw_color = color;
}

void Scribe::set_clear_color(Color color) {
    _clear_color = color;
}

void Scribe::clear(){
    SDL_SetRenderDrawColor(_renderer, _colors[_clear_color].r, _colors[_clear_color].g, _colors[_clear_color].b, _colors[_clear_color].a);
    SDL_RenderClear(_renderer);
}

void Scribe::update(float frameTime) {
    _camera->update(frameTime);
}

void Scribe::draw(){
    SDL_RenderPresent(_renderer);
}


// Camera shennanigans
void Scribe::snap_move_camera(float x, float y) {
    _camera->set_position(x, y);
}

void Scribe::move_camera(float x, float y) {
    _camera->set_position_target(Vector2(x,y));
}

void Scribe::add_move_camera(Vector2 dir) {
    _camera->set_position_target(_camera->get_position() + dir);
}

void Scribe::snap_zoom_camera(float z) {
    _camera->set_zoom(z);
}

void Scribe::zoom_camera(float z) {
    _camera->set_zoom_target(z);
}

Vector2 Scribe::screen_to_world(Vector2 mouse_event_pos) {
    return mouse_event_pos + _camera->get_position();
}

void Scribe::update_mouse(float x, float y) {
    _mouse_world_pos = Vector2(x, y) - _camera->get_position();
    //TODO: add zoom
}

Vector2 Scribe::get_mouse_vec() {
    return _mouse_world_pos;
}

float Scribe::get_mouse_x() {
    return _mouse_world_pos.x();
}

float Scribe::get_mouse_y() {
    return _mouse_world_pos.y();
}

// CREATOR DESTRUCTOR
Scribe::Scribe(SDL_Renderer* renderer) {
    _renderer = renderer;
    _draw_color = Color::WHITE;
    _clear_color = Color::BLACK;

    _camera = new Camera();

    _colors.push_back({ 0x00, 0x00, 0x00, 0xff });   // BLACK
    _colors.push_back({ 0xff, 0xff, 0xff, 0xff });   // WHITE
    _colors.push_back({ 0xff, 0x00, 0x00, 0xff });   // RED
    _colors.push_back({ 0xff, 0xff, 0x00, 0xff });   // YELLOW
    _colors.push_back({ 0x00, 0xff, 0x00, 0xff });   // GREEN
    _colors.push_back({ 0x00, 0xff, 0xff, 0xff });   // CYAN
    _colors.push_back({ 0x00, 0x00, 0xff, 0xff });   // BLUE
    _colors.push_back({ 0xff, 0x00, 0xff, 0xff });   // PURPLE
    _colors.push_back({ 0x00, 0x00, 0x00, 0x00 });   // NONE

    TTF_Init();
    _font = TTF_OpenFont(DEFAULT_FONT, 14);
}

Scribe::~Scribe() {
    TTF_CloseFont(_font);
    TTF_Quit();
    SDL_FreeSurface(_msg_sur);
    SDL_DestroyTexture(_msg_tex);
}