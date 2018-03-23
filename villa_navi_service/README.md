# Villa_navi_service 

* navigation service to pre-defined 
* Localization should be done before using this package
* Set predefined locations in config/naviconfig.yaml
* rosservice
 - villa_navi_service/GoTargetPos
 - Inputs: Desired (x,y,theta) w.r.t map frame. 
* rostopic msg (only for using waypoint)
  - std_msgs::String ("/way_point")
  - "/way_point"

## villa_navi_service server 

```
rosrun villa_navi_service nav_service_test
```

## villa_navi_service client 

```
rosrun villa_navi_service client_test
```
## test waypoint service

````
roslaunch Villa_navi_service service.launch
````

````
rosrun villa_navi_service waypoint_test
````
