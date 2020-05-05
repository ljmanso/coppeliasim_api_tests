# coppeliasim_pythonic_api

This repository provides a Pythonic API for the CoppeliaSim robotics simulator based on their b0 interface.

## Dependencies

```sudo python3 -m pip install msgpack```

## Installation

1. Download CoppeliaSim from [their website](https://www.coppeliarobotics.com/ubuntuVersions) and uncompress the compressed tarball in an appropriate directory.

2. Edit your `.bashrc` file and add the following lines (**make sure you adapt the path in the first of the three lines accordingly**).
After editing the file, make sure that the changes are applied (if you are not sure about how to do that, just log off and log back in).
```
export COPPELIASIM_ROOT=/home/USER/software/coppeliasim
alias coppeliasim="cd $COPPELIASIM_ROOT; bash coppeliaSim.sh"
export B0_RESOLVER="tcp://127.0.0.1:22000"
```

**make sure you adapt the path in the first of the five lines accordingly**

3. The latest version of CoppeliaSim has a couple of minor bugs that we have to fix ([a](https://forum.coppeliarobotics.com/viewtopic.php?f=5&t=8387),[b](https://forum.coppeliarobotics.com/viewtopic.php?t=8378)).
Run the following commands in the command line:
```
cd $COPPELIASIM_ROOT
sed -i 's/function LoadModel(/function LoadModelFromFile(/g' lua/b0RemoteApiServer.lua
rm libsimExtDynamicsVortex.so
```

4. We want to automatically start Coppelia's b0 interface automatically with the simulator.
To do that, use the following commands:
```
cd $COPPELIASIM_ROOT
mv lua/b0RemoteApiServer.lua simAddOnScript_b0RemoteApiServer.lua
```
### Instalation on mac OS

The process is similar to the steps stated previously with the following modifications:

* On latest versions of mac OS the default shell is zsh so you may have to edit `.zshrc` instead of `.bashrc`.

* The setting of environment variables differs a bit from the previous. It has to be at follows:

```
export COPPELIASIM_ROOT=/Users/USER/software/CoppeliaSim
export COPPELIASIM_ROOT_DIR=/Users/USER/software/CoppeliaSim
export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$COPPELIASIM_ROOT_DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT_DIR
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT_DIR
export B0_RESOLVER="tcp://127.0.0.1:22000"
alias coppeliasim="open ${COPPELIASIM_ROOT_DIR}/coppeliaSim.app"
```
* Make sure that you have installed `opencv-python-headless` instead of `opencv-python` and never both at the same time.

* Finally, in macOS Catalina you will have to disable the System Integrity Protection or it won't let you set the global variables DYLD_LIBRARY_PATH and LD_LIBRARY_PATH. [Here](https://developer.apple.com/library/archive/documentation/Security/Conceptual/System_Integrity_Protection_Guide/ConfiguringSystemIntegrityProtection/ConfiguringSystemIntegrityProtection.html) you have a link that explain the process for enable or disable it.

## Description of the API provided

The API provided is composed of one main class and some auxiliary ones.
The main class is called `CoppeliaSimAPI` and provides some methods which are already provided by the b0 version of CoppeliaSim API, plus some additional methods.

The following is a list of the methods which are already provided as part of CoppeliaSim b0 API. If you have any doubts about what they do, please check the [original documentation](https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApi-python.htm):
```
  start(self) ###
  stop(self) ###
  get_objects_children(parent, children_type='sim.handle_all') ###
  get_object_name(object_handle, alternative_name='') ###
  create_model(model, x, y, z, rz) ###
  get_object_handle(object_name) ###
  set_object_position(obj, x, y, z) ###
  set_object_orientation(obj, lst) ###
  remove_object(obj) ###
  run_script(script) ###
  close(self) ###
```

Additionally, the `CoppeliaSimAPI` class provides the following methods:

- `set_object_transform(obj, x, y, z, anglez)` Allows setting the position and orientation using a single call.
- `create_human(x, y, z, angle)` Creates a _cylinder following human_. You can move the human by moving the cylinder.
- `create_wall(p1, p2)` Creates a wall from `p1` to `p2`, where both are lists containing the x, y, z coordinates of the ends of the wall.
- `scale_object(handle, sx, sy, sz)`: Scales the given object, using `sx`, `sy` and `sz` as the ratios.

