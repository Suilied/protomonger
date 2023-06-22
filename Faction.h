#pragma once

class Faction {
	// "owner" class that agents can register to in order to get a working FOF identification system
public:
	Faction* get_faction() { return this; }	// can be used for registering at the agent level
};