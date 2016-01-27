# Usage

## Pirate Tutorials

Set things up:

```
> rocon_launch rocon_gateway_tutorials gateway_tutorials.concert
```

This creates:

* Hub on 11311
* Source of tutorial pubs, subs, services and actions on 11312
* Empty Sink on 11313

To Test:

* Open one shell with master uri port to 11312.
* Open another shell with master uri port to 11313.

```
pirate:11312$ rosrun rocon_gateway_tutorials flip_all.py
pirate:11312# look around
pirate:11312$ rocon_gateway_graph
pirate:11312$ gateway_info
pirate:11312$ remote_gateway_info

pirate:11313# look around and test
pirate:11313$ rostopic list
pirate:11313$ rostopic echo /babbler

pirate:11312# reset things
pirate:11312$ rosrun rocon_gateway_tutorials flip_all.py --cancel
```

You can do the same with `advertise_all.py` and `pull_all.py`. The
`xxx_tutorials.py` scripts operate similarly, but can also work on connections
individually and with regexes.

## Hub Connection Tutorials

### Connect by Param

```
> rocon_launch rocon_gateway_tutorials hub_by_param.concert
```

### Connect by Zeroconf

```
> rocon_launch rocon_gateway_tutorials hub_by_zeroconf.concert
```

### Connect by Service

```
> rocon_launch rocon_gateway_tutorials hub_by_service.concert
```

And in the second master:

```
gateway:11312$ rosrun rocon_gateway_tutorials connect_hub_by_service.py
```
