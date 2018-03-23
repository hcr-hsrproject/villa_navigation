# Villa_navi_service 

* navigation service to pre-defined 
* Localization should be done before using this package
* Set predefined locations in config/naviconfig.yaml
* Rosservice
  - villa_navi_service/GoTargetPos
  - Inputs: Desired (x,y,theta) w.r.t map frame. 
* Rostopic msg (only for using waypoint)
  - std_msgs::String ("/way_point")
  - "/way_point"

## villa_navi_service server 

```
rosrun villa_navi_service nav_service_test
```

## villa_navi_service client test

```
rosrun villa_navi_service client_test
```
## Waypoint service test

````
roslaunch Villa_navi_service service.launch
````

Publish rostopic msg 
````
rostopic pub /way_point std_msgs/String "'data: door'"
````
