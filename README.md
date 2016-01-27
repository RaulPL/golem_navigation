# golem_navigation
Navigation functions for the robot Golem-III.

To run all the navigation functions use:
$ roslaunch golem_navigation golem_navigation.launch map_places:=places.yaml

places.yaml was generated using rqt_topo_map and has to be stored in the resources folder.

### Stand alone gmapping
$ roslaunch golem_navigation golem_gmapping.launch stand_alone:=true

### navigate_service

Examples of possible requests to the navigate_service:

```json
{
    "navigate" : "hall"
}
```
```json
{
    "navigate" : "{x:0.0, y:0.0, angle:0.0}"
}
```
```json
{
    "advance" : 1.0
}
```
```json
{
    "turn" : 45.0
}
```
```json
{
    "advance_fine" : 0.1
}
```
```json
{
    "turn_fine" : 10.0
}
```

And the service returns the status of the navigation task when it is over, which can be "ok" or "navigation_error", according to the status sent by the move_base node. Example:
```json
{
  "status": "ok"
}
```

### swapper_node

The swapper node is in charge of changing the navigation method (amcl or gmapping). This node has the following services:

* start_amcl
* start_gmapping
