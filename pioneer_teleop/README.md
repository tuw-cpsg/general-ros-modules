pioneer_teleop
==============

Contains nodes to steer the Pioneer robot.

This package contains three nodes to control the robot:
* keyboard
* configuration file with twist (speed of translation and rotation)
  and time (how long this twist should be applied)
* specifying a goal pose through an argument or in interactive mode
  (goal pose entered via terminal)


Dependencies
------------

When using the node "goalpose" the node "move_base_simple" of package
"move_base_simple" has to be running too.


Further Information
-------------------

For more details on usage and installation see the wiki page
(https://forge.vmars.tuwien.ac.at/projects/generalrosmodules/wiki/Pioneer_teleop).
