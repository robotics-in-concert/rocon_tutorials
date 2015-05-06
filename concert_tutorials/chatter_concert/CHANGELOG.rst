^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package chatter_concert
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.7 (2015-05-06)
------------------

0.6.6 (2015-02-27)
------------------
* disable zeroconf option
* Contributors: Jihoon Lee

0.6.5 (2015-02-09)
------------------
* revert unused src
* change name of auto enable service argument
* Contributors: dwlee

0.6.4 (2015-01-07)
------------------

0.6.3 (2015-01-05)
------------------

0.6.2 (2014-11-29)
------------------
* updating compatibility of webapps and weburls closes `#50 <https://github.com/robotics-in-concert/rocon_tutorials/issues/50>`_
* [chatter_concert] unit test esoteric names.
* auto_enable_services now use all instead of bool
* Contributors: Daniel Stonier, Jihoon Lee

0.6.1 (2014-08-26)
------------------

0.6.0 (2014-08-26)
------------------
* introducing parameters for link graph
* use proper lists for hubs/concerts now roslaunch can handle it.
* a roslaunch interaction for show and tell
* a roslaunch for chatter interactions.
* update pairing interactions and chatter interactions for testing.
* add Turtle Concert for dudes/dudettes, useful for testing.
* small bugfixes due to pairing update.
* update concert client launchers for chatter and pairing tutorials.
* update for new web url interactions
* conductor graph now needs a console arg for running in a shell.
* revert back to 2-4 dudes in chatter concert.
* add monitoring shells for the chatter concert rocon launch.
* a conductor graph for easy testing of conductor upgrades.
* local machine args for babbler and turtle concert, see also `#36 <https://github.com/robotics-in-concert/rocon_tutorials/issues/36>`_.
* usable configuration for service priorities.
* update the android interaction
* local machine args for switching.
* rocon_service -> concert_service
* revert the flexibility, not possible because nested subst variables (find outside of an arg) in the concert master launcher.
* update to new concert arg format.
* override -> overrides
* keep chatter concert simple, create babbler concert as an illustrative customisation.
* parameterisation and override experiments, solution bundling folders added.
* bugfix logo location.
* rename resource to resource_name
* solution parameter tests
* add services file for solution
* update concert format to load services file
* switch to icons in rocon_icons.
* doc interactions for the chatter and turtle concerts
* bugfix the chatter qt program.
* upgrade interactions for chatter concert.
* moved services to new rocon_services repo.
* default to a central admin service.
* chatter has its own qt listener
* shadow admin service for chatter concert.
* adjustments to drop heir-part of uri if no concert name
* upgrade to service exports instead of service lists.
* chatter and turtle concerts upgraded for the new rocon uri.
* condense interactions list.
* concert_service_roslaunch -> concert_service_link_graph.
* platform tuple overhaul.
* update for exported rapp specifications.
* environment variables for selecting the scheduler/requester (beware incomptaibile combinations)
* use an environment variable optionally here.
* forgot to switch the requester for these...closes `#17 <https://github.com/robotics-in-concert/rocon_tutorials/issues/17>`_
* open the range, default to demo scheduler.
* use compatibility tree for chatter concert.
* minor modifications, also updated chatter and turtle concerts.
* titles for chatter and turtle concerts.
* deprecate the old platform info message.
* updates for legacy master.
* parameterisation for local remote control of clients only.
* Contributors: Daniel Stonier, Jihoon Lee

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
* update to handle fix for app absolute remappings
* updated for dynamic handling joing and leaving.
* testies with dynamic client handling.
* new test configuration scenario for dynamic client handling.
* app list to rapp list
* rocon_orchestra -> concert_orchestra
* removed non shared includes.
* utilising the new conductor launcher and renaming included launchers.
* 0.4.0
* 0.3.4
* bringing up to speed with groovy-devel branch.

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
* no longer installing the apps directory.
* .app -> .rapp
* no more concert client, also added metapackage to metapackage dependencies.
* auto start flag.
* some moving around, also chatter_concert.
