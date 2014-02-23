These were some old scripts that were getting used to try and write a rocon_launch style spawner (i.e. spawns terminals).

They fail on precise though because of a bug in konsole that doesn't let it fork properly. This however is
fixed on saucy so we can move the rocon_launched turtle clients to turtlesim service spawned clients when ready.