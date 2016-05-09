pioneer_teleop
==============

This package contains a node to control the robot:

* keyboard

keyboard reads from stdin (console) within a separate thread. Note
that this node can only be terminated by pressing 'q'. Use
'w','a','s','d' and space to control the robot.


Dependencies
------------

None, but of course a driver for your robot should be running too.
