#pragma once

#include "Vector2.h"

#ifndef PIE
#define PIE 3.14159265358979323846f
#endif

#define DDELTA .1f

static float QuadIn(float t) { return t * t; }
static float QuadOut(float t) { return 2.f * t - t * t; }
static float QuadInOut(float t) { return (t <= 0.5f) ? (t * t * 2.f) : (-1.f + 4.f * t + -2.f * t * t); }

static float CubeIn(float t) { return t * t * t; }
static float CubeOut(float t) { return 1.f - CubeIn(1.f - t); }
static float CubeInOut(float t) { return (t <= 0.5f) ? CubeIn(t * 2.f) * 0.5f : CubeOut(t * 2.f - 1.f) * 0.5f + 0.5f; }

static float BackIn (float t) { return t * t * (2.70158f * t - 1.70158f); }
static float BackOut (float t) { return 1.f - BackIn(1.f - t); }
static float BackInOut (float t) { return (t <= 0.5f) ? BackIn(t * 2.f) * 0.5f : BackOut(t * 2.f - 1.f) * 0.5f + 0.5f; }

static float ExpoIn (float t) { return std::pow(2.f, 10.f * (t - 1.0f)); }
static float ExpoOut (float t) { return 1.f - std::pow(2.f, -10.f * t); }
static float ExpoInOut (float t) { return t < .5f ? ExpoIn(t * 2.f) * 0.5f : ExpoOut(t * 2.f - 1.f) * 0.5f + 0.5f; }

static float SineIn (float t) { return -std::cos(PIE * 0.5f * t) + 1.f; }
static float SineOut (float t) { return std::sin(PIE * 0.5f * t); }
static float SineInOut (float t) { return -std::cos(PIE * t) * 0.5f + .5f; }

static float ElasticOut(float t) { return std::pow(2.f, -10.f * t) * std::sin((t - 0.075f) * (2.f * PIE) / 0.3f) + 1.f; }
static float ElasticIn (float t) { return 1.f - ElasticOut(1.f - t); }
static float ElasticInOut (float t) { return (t <= 0.5f) ? ElasticIn(t * 2.f) / 2.f : ElasticOut(t * 2.f - 1.f) * 0.5f + 0.5f; }

static float lerp(float start, float end, float t) {
    return (start * (1.0f - t)) + (end * t);
}

class Camera {
private:
    // position (start at 0,0)
    Vector2 _position;
    Vector2 _position_target;
    bool _do_lerp;
    float _lerp_time;
    float _lerpx;
    float _lerpy;

    // zoom (none  for now)
    float _zoom;
    float _zoom_target;

    // delta values that indicate the (max)speed at which the camera will zoom or move
    float _d_position;
    float _d_zoom;

public:
    // GETTERS
    Vector2 get_position() { return _position; }
    float get_zoom() { return _zoom; }

    // SETTERS
    void set_position(float x, float y) {
        _position = Vector2(x, y);
        _position_target = _position;
    };

    void set_position_target(Vector2 pos) {
        _position_target = pos;
        _do_lerp = true;
        _lerp_time = .0f;
        _lerpx = _position.x();
        _lerpy = _position.y();
    }

    void set_zoom(float z) {
        _zoom = z;
        _zoom_target = _zoom;
    }

    void set_zoom_target(float z) {
        _zoom_target = z;
    }

    void set_zoom_speed(float d) {
        _d_zoom = d;
    }

    void set_move_speed(float d) {
        _d_position = d;
    }

    // update the camera's position / zoom each frame according to pos and zoom targets
    void update(float frameTime) {
        if (_do_lerp) {
            float posx = lerp(_lerpx, _position_target.x(), QuadInOut(_lerp_time));
            float posy = lerp(_lerpy, _position_target.y(), QuadInOut(_lerp_time));
            _position.set(posx, posy);
            if (_lerp_time > 1.f)
                _do_lerp = false;
            _lerp_time += frameTime;
        }
    };

    Camera() {
        _position = Vector2();
        _zoom = 1.0f;
        _position_target = _position;
        _zoom_target = _zoom;
        _do_lerp = false;

        _d_position = DDELTA;
        _d_zoom = DDELTA;
    };
    ~Camera();
};