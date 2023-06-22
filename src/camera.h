#pragma once

#include "Vector2.h"

class Camera {
private:
    // position (start at 0,0)
    Vector2 _position;
    Vector2 _position_target;

    // zoom (none  for now)
    float _zoom;
    float _zoom_target;

    // delta values that indicate the (max)speed at which the camera will zoom or move
    float _d_position;
    float _d_zoom;

public:
    void set_position(float x, float y) {
        _position = Vector2(x, y);
        _position_target = _position;
    };

    Vector2 get_position() {
        return _position;
    }

    void set_position_target(Vector2 pos) {
        _position_target = pos;
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
    void update() {
        if (_position_target != _position) {
            Vector2 move_direction = _position - _position_target;
            move_direction = normalize(move_direction) * _d_position;
            if (absSq(move_direction) <= _d_position)
                _position = _position_target;
            else
                _position += move_direction;
        }

        if (_zoom != _zoom_target) {                            // do we need to adjust zoom?
            float zoom_direction = _zoom - _zoom_target;        
            float zoom_direction_actual = zoom_direction < 0 ?  // get the actual stepsize in the correct direction
                std::fmaxf(zoom_direction, -1.0f * _d_zoom) :
                std::fminf(zoom_direction, _d_zoom);
            if (std::abs(zoom_direction_actual) <= _d_zoom)     // adjust zoom and set if we need to 
                _zoom = _zoom_target;
            else
                _d_zoom += zoom_direction_actual;
        }
    };

    Camera() {
        _position = Vector2();
        _zoom = 1.0f;
        _position_target = _position;
        _zoom_target = _zoom;

        _d_position = .1f;
        _d_zoom = .1f;
    };
    ~Camera();
};