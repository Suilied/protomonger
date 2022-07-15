#include "circlepacker.h"

int CirclePacker::add_circle(float radius) {
	_circles.push_back(radius);
	return _circles.size();
}

CircleNode* CirclePacker::new_circle_node(int id, float radius) {
	CircleNode* newNode = new CircleNode();
	newNode->radius = radius;
	newNode->id = id;

	return newNode;
}

void CirclePacker::set_center(float x, float y) {
	_pack_center = Vector2(x, y);
}
void CirclePacker::set_center(Vector2 pos) {
	_pack_center = pos;
}

Vector2 CirclePacker::get_center() {
	return _pack_center;
}

void CirclePacker::bound(CircleNode* n, Vector2* bb_topright, Vector2* bb_bottomleft) {
	float bleftx = std::min(n->x - n->radius, bb_bottomleft->x());
	float blefty = std::min(n->y - n->radius, bb_bottomleft->y());
	float trightx = std::max(n->x + n->radius, bb_topright->x());
	float trighty = std::max(n->y + n->radius, bb_topright->y());
	bb_bottomleft->set(bleftx, blefty);
	bb_topright->set(trightx, trighty);
}

void CirclePacker::place(CircleNode* a, CircleNode* b, CircleNode* c) {
	float da = b->radius + c->radius;
	float db = a->radius + c->radius;
	float dx = b->x - a->x;
	float dy = b->y - a->y;
	float dc = std::sqrt((dx * dx) + (dy * dy));
	if (dc > 0.f) {
		float val_to_cos = ((db * db) + (dc * dc) - (da * da)) / (2 * db * dc);
		float theta = std::acos(val_to_cos);
		float x = val_to_cos * db;
		float h = std::sin(theta) * db;
		dx = dx / dc;
		dy = dy / dc;
		c->x = a->x + (x * dx) + (h * dy);
		c->y = a->y + (x * dy) - (h * dx);
	}
	else {
		c->x = a->x + db;
		c->y = a->y;
	}
}

float CirclePacker::distance(CircleNode* n) {
	Vector2 dist_n_vec = Vector2(n->x, n->y);
	return std::sqrt(dist_n_vec * dist_n_vec);
}

bool CirclePacker::intersects(CircleNode * a, CircleNode * b) {
	float dx = b->x - a->x;
	float dy = b->y - a->y;
	float dr = a->radius + b->radius;
	float comp0 = (dr * dr) - 1e-6;
	float comp1 = (dx * dx) + (dy * dy);
	return comp0 > comp1;
}

void CirclePacker::splice(CircleNode* a, CircleNode* b) {
	a->next = b;
	b->prev = a;
}

void CirclePacker::insert(CircleNode* a, CircleNode* b) {
	CircleNode* c = a->next;
	a->next = b;
	b->prev = a;
	b->next = c;
	if (c) {
		c->prev = b;
	}
}

void CirclePacker::pack() {
	// first clean up the old packing info
	for (int i = 0; i < _circle_nodes.size(); i++) {
		delete _circle_nodes[i];
	}
	_circle_nodes.clear();

	CircleNode* first_node = NULL;
	CircleNode* last_inserted_node = NULL;
	Vector2 bb_bottomleft = Vector2(INT_MAX, INT_MAX);
	Vector2 bb_topright = Vector2(INT_MIN, INT_MIN);

	float size = 0.f;
	 
	for (int i = 0; i < _circles.size(); i++) {
		CircleNode* new_node = new_circle_node(i, _circles[i]);
		if (first_node == NULL) {
			first_node = new_node;
		}
		else {
			last_inserted_node->insert_next = new_node;
		}
		last_inserted_node = new_node;
		_circle_nodes.push_back(new_node);
	}

	// lets pack some circles place_destinations(first_node, bbTR, bbBL, _pack_direction);
	CircleNode* a = first_node;
	CircleNode* b = NULL;
	CircleNode* c = NULL;

	if (first_node == NULL) {
		// this should never happen
		return;
	}

	Vector2 a_offset = -(_pack_direction * a->radius);
	a->x = a_offset.x();
	a->y = a_offset.y();
	bound(a, &bb_topright, &bb_bottomleft);

	if (!a->insert_next) {
		return; // (bb_topright - bb_bottomleft);
	}

	b = a->insert_next;
	Vector2 b_offset = _pack_direction * b->radius;
	b->x = b_offset.x();
	b->y = b_offset.y();
	bound(b, &bb_topright, &bb_bottomleft);

	if (!b->insert_next) {
		return; // (bb_topright - bb_bottomleft);
	}

	c = b->insert_next;
	place(a, b, c);
	bound(c, &bb_topright, &bb_bottomleft);

	if (!c->insert_next) {
		return; // (bb_topright - bb_bottomleft);
	}

	a->next = c;
	a->prev = b;
	b->next = a;
	b->prev = c;
	c->next = b;
	c->prev = a;
	b = c;
	c = c->insert_next;
	bool skip = false;

	while (c) {
		if (!skip) {
			CircleNode* n = a;
			CircleNode* nearest_node = n;
			float nearest_distance = 1000000.0f;

			float dist_n = distance(n);
			if (dist_n < nearest_distance) {
				nearest_distance = dist_n;
				nearest_node = n;
			}
			n = n->next;

			while (n != a) {
				dist_n = distance(n);
				if (dist_n < nearest_distance) {
					nearest_distance = dist_n;
					nearest_node = n;
				}
				n = n->next;
			}

			a = nearest_node;
			b = nearest_node->next;
			skip = false;
		}

		place(a, b, c);

		bool intersected = false;
		CircleNode* j = b->next;
		CircleNode* k = a->prev;
		float sj = b->radius;
		float sk = a->radius;

		if (sj <= sk) {
			if (intersects(j, c)) {
				splice(a, j);
				b = j;
				skip = true;
				intersected = true;
			}
			sj += j->radius;
			j = j->next;
		}
		else {
			if (intersects(k, c)) {
				splice(k, b);
				a = k;
				skip = true;
				intersected = true;
			}
			sk += k->radius;
			k = k->prev;
		}

		while (j != k->next) {
			if (sj <= sk) {
				if (intersects(j, c)) {
					splice(a, j);
					b = j;
					skip = true;
					intersected = true;
					break;
				}
				sj += j->radius;
				j = j->next;
			}
			else {
				if (intersects(k, c)) {
					splice(k, b);
					a = k;
					skip = true;
					intersected = true;
					break;
				}
				sk += k->radius;
				k = k->prev;
			}
		}

		if (!intersected) {
			insert(a, c);
			b = c;
			bound(c, &bb_topright, &bb_bottomleft);
			skip = false;
			c = c->insert_next;
		}
	}

	// set the root node
	_root = a;

	// bonus, try and approximate the packed diameter
	_approx_diameter = abs(bb_topright - bb_bottomleft);
}

float CirclePacker::get_approx_diameter() {
	return _approx_diameter;
}

void CirclePacker::set_circle_radius(int id, float radius) {
	_circle_nodes[id]->radius = radius;
	_circles[id] = radius;
}

Vector2 CirclePacker::get_circle_position(int id) {
	CircleNode* req_node = _circle_nodes[id];
	return Vector2(req_node->x + _pack_center.x(), req_node->y + _pack_center.y());
}

void CirclePacker::clear() {
	for (int i = 0; i < _circle_nodes.size(); i++) {
		delete _circle_nodes[i];
	}
	_circle_nodes.clear();
	_circles.clear();
	_root = NULL;

	// lets not clear these values
	//_pack_center = Vector2();
	//_pack_direction = Vector2(1, 0);
}

CirclePacker::CirclePacker() {
	_pack_center = Vector2();
	_pack_direction = Vector2(1, 0);
	_root = NULL;
}
CirclePacker::~CirclePacker() {
	clear();
}