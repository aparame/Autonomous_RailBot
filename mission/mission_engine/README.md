This branch contains all the files for the mission engine.

## Common prep tasks
1. As with most catkin_make workflows, you have to run catkin_make inside the catkin_ws directory.
2. Be sure to run ``source devel/setup.bash`` to set up the environment variables.
3. You can launch the nodes using the command ``roslaunch launch/master.launch``
4. Use `mission_engine/missions/missionwriter.py` to write the `mission*.txt` in the same folder
5. Under Linux, remember to use `chmod +x` to convert all script python files to be executable

The following sequence of commands can be used to load and execute missions on the bot.

1. To the topic host2ros (also accessible through the HMI), publish the command ``load mission0.txt``
2. To the topic host2ros, publish the command ``execute`` to run the whole mission or the command ``singlestep`` to run the mission command-by-command.
3. The execution moves from one command to the next only when it sees a Bool published on the topic ``cmd_done`` (from the motor control node.)
4. The mission rewriter will automatically split the programs when an obstacle is encountered. For example, in a single command mission ``move +10``, if the bot
encounters an obstacle and stops. If the obstacle were to be moved after the bot stopped at 7m (say), the program is rewritten to add a new command ``move +3`` to restore
the semantics of the original mission.
5. The mission rewriter subscribes to the `string_lidar/obstacle` topic and the ``pose/pose/position/x`` topic to handle restarts correctly.
6. Finally, the mission engine publishes a Boolean topic ``mission_active`` to indicate to the motor controller when it should listen for motor movement commands.
