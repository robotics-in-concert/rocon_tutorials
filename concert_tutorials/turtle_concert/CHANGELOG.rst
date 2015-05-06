^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtle_concert
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.7 (2015-05-06)
------------------

0.6.6 (2015-02-27)
------------------
* default values for request turtle closes `#58 <https://github.com/robotics-in-concert/rocon_tutorials/issues/58>`_
* Contributors: Jihoon Lee

0.6.5 (2015-02-09)
------------------
* change name of auto enable service argument
* Contributors: dwlee

0.6.4 (2015-01-07)
------------------
* install turtle_pony.py closes `#52 <https://github.com/robotics-in-concert/rocon_tutorials/issues/52>`_
* Contributors: Jihoon Lee

0.6.3 (2015-01-05)
------------------

0.6.2 (2014-11-29)
------------------
* updating compatibility of webapps and weburls closes `#50 <https://github.com/robotics-in-concert/rocon_tutorials/issues/50>`_
* auto_enable_services now use all instead of bool
* Contributors: Jihoon Lee

0.6.1 (2014-08-26)
------------------

0.6.0 (2014-08-26)
------------------
* relative path rapps
* rename teleop to video_teleop
* turtle concert parameter ready
* add missing turtlesim parameters.
* build include path 'include' does not exist
* full launcher configuration via service parameters.
* turtle2 moving in, closes `#39 <https://github.com/robotics-in-concert/rocon_tutorials/issues/39>`_.
* indigo updates for queue_size.
* use the new preemptive_compatibility_tree scheduler declaration
* rocon_qt_teleop tutorial for the rocon app manager.
* update for new web url interactions
* remove the 10s sleep for turtle pond service.
* local machine args for babbler and turtle concert, see also `#36 <https://github.com/robotics-in-concert/rocon_tutorials/issues/36>`_.
* dynamic canceling and requesting to support preemptible turtles.
* unnecessary priority listing in turtle pond service removed.
* usable configuration for service priorities.
* rocon_scheduler_requests -> concert_scheduling/concert_scheduler_requests
* rocon_service -> concert_service
* turtle2 with turtlesim2 concert.
* configurable scheduler from the command line.
* bugfix scheduler typo -> compatibility_tree
* turtle pond now a ros (not a link graph) service and uses the scheduler switcher.
* integrating jack's scheduler into the tutorials.
* teleop now uses parent rapp.
* Customise the concert name for the turtle teleop concert.
* revert the flexibility, not possible because nested subst variables (find outside of an arg) in the concert master launcher.
* update to new concert arg format.
* use the central icons pack.
* parameterisation and override experiments, solution bundling folders added.
* typo fix, rocon_services_admin -> rocon_service_admin
* head shot to hopefully the last rocon_utilities reference
* rename resource to resource_name
* service argument now accpes solution configuration instead of list of services
* bugfix typos in service list and fix turtle pond interactions file.
* some helpful comments
* solution parameter tests
* rename turtle to turtle_pond for consistency and compatibility
* add services file for solution
* update concert format to load services file
* switch to icons in rocon_icons.
* doc interactions for the chatter and turtle concerts
* remove now unused rocon_utilities dependency.
* rename interface into public_interface
* fix typo pointing to admin service in turtle concerts.
* turtle teleop concert for testing.
* turtle_concert/teleop rapp.
* moved services to new rocon_services repo.
* fix the spawning turtles from spawning - problem was in flipping two
  topics from separate clients with the same node names.
* comment about the relay, also remove the debug service pair client.
* stop chaining ros service calls (creates timeouts everywhere). Still some problems spawning elsewhere though - python service pairs?
* rename turtles service to turtle pond.
* upgraded turtlesim tutorial.
* flip pose as well.
* flip across the engine room to the hatchling.
* hatchling now flipping everything correctly.
* continued work on the turtles for turtlesim.
* collapse turtle launchers using rocon launcher args.
* turtle herder script working
* spawn testing.
* more work on the herder
* start upgrading turtlesim
* default to a central admin service.
* adjustments to drop heir-part of uri if no concert name
* upgrade to service exports instead of service lists.
* chatter and turtle concerts upgraded for the new rocon uri.
* concert_service_roslaunch -> concert_service_link_graph.
* platform tuple overhaul.
* make use of required roslaunch tag in rapps.
* update for exported rapp specifications.
* remove legacy references to concert_orchestra.
* environment variables for selecting the scheduler/requester (beware incomptaibile combinations)
* forgot to switch the requester for these...closes `#17 <https://github.com/robotics-in-concert/rocon_tutorials/issues/17>`_
* fix install rules.
* minor modifications, also updated chatter and turtle concerts.
* turtle concert compatible for both. old and new style of concert
* titles for chatter and turtle concerts.
* deprecate the old platform info message.
* updates for legacy master.
* parameterisation for local remote control of clients only.
* Contributors: Daniel Stonier, Jihoon Lee, dwlee

0.5.6 (2013-08-30)
------------------

0.5.5 (2013-08-07)
------------------

0.5.4 (2013-07-18)
------------------
* rapp list path fix
* 0.5.3
* 0.5.2
* updating run depends

0.5.3 (2013-06-04)
------------------
* reverting twist to turtle velocity in groovy

0.5.2 (2013-05-28)
------------------
* updating run depends
* 0.5.1
* install concert directory
* 0.5.0

0.5.1 (2013-05-27 11:49)
------------------------
* install concert directory

0.5.0 (2013-05-27 11:09)
------------------------
* update remap rules for fixed namespacing in app manager.
* app list to rapp list
* rocon_orchestra -> concert_orchestra
* turtle_concert upgraded for multihub improvements.
* 0.4.0
* 0.3.4
* bringing up to speed with groovy-devel branch.
* turtle_stroll uses geometry_msgs. command_velocity to cmd_vel. removing wrong setup.py

0.3.5 (2013-04-09)
------------------

0.3.4 (2013-04-08)
------------------
* the real 0.3.4

0.3.3 (2013-04-07 23:11)
------------------------

0.3.2 (2013-04-07 23:08)
------------------------

0.3.1 (2013-04-07 23:06)
------------------------

0.3.0 (2013-04-07 21:34)
------------------------
* .app -> .rapp
* update jihoon email
* no more concert client, also added metapackage to metapackage dependencies.
* auto_start option added.
* some moving around, also chatter_concert.
