# mobot_coord
This package defines the service message: MobotCoord.srv
This package can also be used to construct the robot coordinator.
An example skeleton is included here.

Services within other packages that need to communicate with the coordinator should include a dependency on this package, "mobot_coord"


## Example usage
First, start up the services.  At present, dummy services are defined in the package mobot_services.
Start a roscore, then run these three services from separate terminals:
`rosrun mobot_services dummy_percep_svc'
`rosrun mobot_services dummy_nav_svc'
`rosrun mobot_services dummy_manip_svc'

These nodes present services by the names of "perception_service", "navigation_service" and "manipulation_service", all of which use
the same service message "Mobot_coord.srv", which is defined in this (mobot_coord) package.


After starting the 3 services, run this dummy coordination node:
`rosrun mobot_coord dummy_robot_coordinator'

Observe the outputs in each window.

    
