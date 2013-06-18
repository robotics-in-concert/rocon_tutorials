rocon_multirobot_experimental
=============================

Multi robot visualization and navigation testing repo. Disabled with a CATKIN_IGNORE for now since
all the rosinstallers are broken and this should(?) perhaps be better placed on its own because of
the huge dependency hit.

* Works in Hydro!!!

#### Pre-requisite

Prepare environment for [ros-groovy-desktop-full](http://ros.org/wiki/groovy/Installation/Ubuntu).

```
sudo apt-get install ros-groovy-turtlebot
sudo apt-get install ros-groovy-turtlebot-apps
sudo apt-get install ros-groovy-turtlebot-viz
sudo pip install -U yujin_tools
```


#### Installation ####

You can use catkin_workspace, but [yujin_tools](https://github.com/yujinrobot/yujin_tools/wiki/yujin-init)
makes life easier.

```
> yujin_init_workspace --track=hydro ~/multinav multinav
> cd ~/multinav
# Using groovy underlay...for now
> rosdep install --from-paths src /opt/ros/groovy --ignore-src --rosdistro groovy -y
> yujin_init_build . /opt/ros/groovy
> yujin_make
```

#### Execution ####

* Simple way to launch in one computer

```
> source ~/multinav/devel/setup.bash
> rocon_launch multinav_concert multinav.concert
> rosservice call /concert/start_solution
```

* In case simple way does not work
  * Don't forget to source the correct setup.bash
  * Run each command separate terminal

```
> roslaunch multinav_concert concert.launch --port=11311
> roslaunch multinav_database multinav_database_client.launch --port=11312
> roslaunch soft_navbot_client.launch --port=11313
> roslaunch soft_navbot_client.launch --port=11314
> roslaunch soft_navbot_client.launch --port=11315
> rosservice call /concert/start_solution
```

#### Navigation ####

* Softbot  - /softbot/goal_simple
* Softbot2 - /softbot2/goal_simple
* Softbot3 - /softbot3/goal_simple

#### RQT graph ####

```
# In central workspace.
> rocon_gateway_graph
```
