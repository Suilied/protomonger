#pragma once

#include <vector>
#include "Vector2.h"

class CircleNode {
private:
	float x;
	float y;
	Vector2 position;
	float radius;
	int id;
	CircleNode* next;
	CircleNode* prev;
	CircleNode* insert_next;

	void set_destination(float r, CircleNode* n, CircleNode* p) {
		radius = r;
		next = n;
		prev = p;
	}

	CircleNode() {
		x = 0.f;
		y = 0.f;
		position = Vector2();
		radius = 0.f;
		id = 0;
		next = NULL;
		prev = NULL;
		insert_next = NULL;
	}
	~CircleNode() {}

	friend class CirclePacker;
};

class CirclePacker {
private:
	Vector2 _pack_center;
	Vector2 _pack_direction;
	std::vector<CircleNode*> _circle_nodes;	// map of previously allocated _circles, this way we can reuse 
	std::vector<float> _circles;	// array of circles (diameters in float) that need to be packed
	CircleNode* _root;

	CircleNode* new_circle_node(int id, float radius);
	void bound(CircleNode* n, Vector2* bb_topright, Vector2* bb_bottomleft);
	void place(CircleNode* a, CircleNode* b, CircleNode* c);
	float distance(CircleNode* n);
	bool intersects(CircleNode* a, CircleNode* b);
	void splice(CircleNode* a, CircleNode* b);
	void insert(CircleNode* a, CircleNode* b);

public:
	int add_circle(float radius);
	void set_center(float x, float y);
	void set_center(Vector2 pos);
	Vector2 get_center();

	void pack();

	void set_circle_radius(int id, float radius);
	Vector2 get_circle_position(int id);
	void clear();

	CirclePacker();
	~CirclePacker();

	friend class CircleNode;
};
