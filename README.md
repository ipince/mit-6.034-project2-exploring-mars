mit-6.034-project2-exploring-mars
=================================

Project 2 of MIT AI class (6.034).

The project consisted of planning a rover's mission in Mars (like Curiosity's, but simpler :) ). Simply put: given a set of goals and constraints (e.g. collect samples at a set of locations, communicate with Earth every so often, not let the battery run out), and a set of actions the rover can perform (e.g. move, drill, recharge, communicate), come up with a plan (a sequence of actions) for the rover that meets the goals/constraints.

The search space of all possible plans is huge and must be explored efficiently. The solution uses A* search with a set of heuristics based on distance (to drill sites), battery, communication, and waiting time. Full details are explained in the report at proj2-paper.pdf.

Main code is at src/proj2.scm. Other .scm files were provided by the teaching staff.