pioneer_teleop
==============

This package contains a node to control the robot:

* keyboard

keyboard reads from stdin (console) within a seperate thread. Note
that this node can only be terminated by pressing 'q'. Use
'w','a','s','d' and space to control the robot.


Dependencies
------------

None. But of course a driver for your robot should be running too.


Further Information
-------------------

For more details on usage and installation see the wiki page
(https://forge.vmars.tuwien.ac.at/projects/generalrosmodules/wiki/Pioneer_teleop).


Changelog
---------

[0.1.0] - 2015-07-15

* remove goalpose and twistfortime nodes

[0.0.1] - 2014-09-02

* changes to work with p2os too

[0.0.1] - 2013-08-14

* initial version