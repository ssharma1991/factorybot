# robot-models

This repo includes Robot and Environment models to be used with Gazebo. The included Gazebo world is shown below

![Image of the Gazebo world](images/my_world.png)

# Gazebo Tips:

# Setting Models, Plugins & Worlds Path
```
export GAZEBO_MODEL_PATH=/home/shashank/Documents/robot-models/model
export GAZEBO_PLUGIN_PATH=/home/shashank/Documents/robot-models/build
export GAZEBO_RESOURCE_PATH=/home/shashank/Documents/robot-models/world
```
Check if the paths were set correctly using `env|grep -i gazebo`

# Compile Plugins
```
cd /home/shashank/Documents/robot-models/build
cmake ../ && make
```

# Open the Gazebo World
```
cd /home/shashank/Documents/robot-models/world
gazebo AGVs --verbose
gazebo building_with_AGVs --verbose
gazebo factory_with_AGVs --verbose

```
If previous instances of gzserver have not ended gracefully, the command might not work. Use `killall -9 gzserver` and try again.
