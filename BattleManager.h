#pragma once

#include <vector>

class Province {
	int provinceId;	// identifier on the world-map
	// owner
	// ground-type
	// heat-level
	// weather
	// active-effects
};

struct Weapon {
	int id;
	int damage;		// the damage value
	int weight;		// modifier to stamina use while wielding the weapon
	// damage type
	// weapon length or, who can hit whom, first and without retaliation
	// extra weapon effect (magic, poison damage, spellcast on swing, etc.)

	// name
	// graphic
};

struct Armor {
	int id;
	int prot;			// the protection value
	int encumbrance;	// modifier to stamina use while wearing the armor
	// protection type
	// 
	// name
	// graphic
};

class Unit {
	int id;
	int regimentId;	// used for cohesion, morale checks, phalanx bonus, coordinated movement
	float x;
	float y;

	//Squad* _squad;
	bool is_squadleader;

	//Army* _army;
	bool is_armyleader;

	//Objective* _objective; // the thing this unit is trying to achieve

	int hp;			// health, if 0 unit is dead
	int sta;		// stamina, if 0 unit is exhausted and can't fight properly
	int speed;		// movement speed of the unit
	int physProt;	// physical protection, physical attacks are negated by this amount
	int magicProt;	// magical protection, magical attacks are negated by this amount
	int defense;	// avoidance 
	std::vector<Weapon> _weapons;	// any armament the unit carries
	std::vector<Armor> _armor;		// any armor the unit wears/carries
	// special attributes (Sacred, any magic paths, research value, swamp-survival, etc.)
};

// if we ever want names for this collection of soldiers that would match
// themetically, i.e. for different factions or different types of troops
// just look up "company" on thesaurus.com
class Squad {					// sub-group of the army
	Unit* _commander;			// the unit that leads this squad
	std::vector<Unit*> _units;
};

class Army {
	int ownerId;
	Unit* _general;				// leader of the army
	std::vector<Squad*> _regiments;
	std::vector<Unit*> _units;
};

class BattleManager {
private: 
	void PrepareBattlefieldData();

	std::vector<Army*> _armies;

public:
	void SetupTestData();
};