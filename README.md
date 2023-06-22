Prototype Warmonger
Mainly a proof of concept and combination of the following techs/libraries:
- RVO
- Flocking algorythm
- Circlepacker
- Pathfinder

Todo Shortlist:
- make it so the path tool can be used to asign movement actions to agents
- implement "terrain" for agents to avoid
- remove SDL_ttf and replace text drawing with sprite-text
- add sprites for the "units"

- build the actual Godot interface
- - Add agents
- - Remove agents
- - get/set agent position, radius, [prefered velocity]
- - - Bulk get/set (for speed)
- - denote agent group
- - set path for agent group

- - Add terrain
- - Remove terrain
- - Run RVOsim step
