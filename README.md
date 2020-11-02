# robot-models

This repo includes Robot and Environment models to be used with Gazebo

# Gazebo Tips:

# Setting Models, Plugins & Worlds Path
```
export GAZEBO_MODEL_PATH=/home/shashank/Documents/robot-models/model
export GAZEBO_PLUGIN_PATH=/home/shashank/Documents/robot-models/script
export GAZEBO_RESOURCE_PATH=/home/shashank/Documents/robot-models/world
```
Check if the paths were set correctly using `env|grep -i gazebo`

# Compile Plugins
```
cd /home/shashank/Documents/robot-models/build
cmake ../ && make
```

# Open the Gazebo Simulation
```
cd /home/shashank/Documents/robot-models/world
gazebo my_world --verbose

```
