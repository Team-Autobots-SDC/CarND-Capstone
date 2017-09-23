rostopic pub -1 /twist_cmd geometry_msgs/TwistStamped  "{twist: {linear:  {x: $1, y: 0.0, z: 0.0}, angular: {x: -0.0,y: 0.0,z: 0.0}}}"
