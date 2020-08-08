# F110 Team 2 Code Base

Code written in collaboration with David DePauw and Brandom McBride

# Dependencies
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [osqp](http://osqp.readthedocs.io/en/latest/index.html)
- [osqp-eigen](https://robotology.github.io/osqp-eigen/doxygen/doc/html/index.html)

You can run the team2_setup.sh script to install osqp and osqp-eigen. This script downloads the source code in a new folder in the home directory. Then, it builds and installs these libraries. Sudo permission is needed in order to run this script.

# Milestone 2 Launch Instructions
For milestone 2, we are using our team's pure pursuit code. We used follow the gap on a modified skirkanich map to get the waypoints.

Once the docker file is running, enter this command in the terminal to run the agent:
```bash
$ roslaunch team2_pure_pursuit team2_pure_pursuit.launch
```

# Milestone 3 Launch Instructions
For milestone 3 we used RRT* for obstacle avoidance.

Once the docker file is running, enter this command in the terminal to run the agent:
```bash
$ roslaunch team2_milestone3 team2_milestone3.launch
```

# Milestone 4 Launch Instructions
For milestone 4 we used RRT* for obstacle avoidance and avoid the opponent's predicted path and MPC to control the car.

Once the docker file is running, enter this command in the terminal to run the agent:
```bash
$ roslaunch rrt_mpc rrt_mpc.launch
```
