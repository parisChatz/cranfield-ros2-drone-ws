# Should Gazebo be launched inside the DroneEnv constructor?
No, it’s not ideal to launch the simulation in the environment’s constructor. Here’s why:

## Downsides of launching from constructor:
1. Tight coupling: It blurs the separation of concerns. The environment's job is to interface with Gazebo, not control the sim lifecycle.
2. Harder to test/debug: Restarting the env might restart Gazebo unnecessarily.
3. Concurrency issues: If multiple environments are launched (e.g., for parallel training), spawning multiple Gazebo instances can crash or conflict with ports/resources.


# Issues faced
## Venv and shit.
For new ubuntu things are only installed in venv. So have to do venvs for sb3 and the rest, but ros gets fucked.

### Solution
Create your .venv with access to system-wide packages, like this:
```python3 -m venv .venv --system-site-packages```
This gives the venv access to the ROS 2 Python packages and bindings that were installed system-wide (e.g., via apt).


## Check if topics exist to complete simulation startup
When I  do ros2 launch <my launch files> I don't see all the gazebo topics that come from plugins inside the model sdf (like the plugin <plugin filename="gz-sim-multicopter-control-system" name="gz::sim::systems::MulticopterVelocityControl"> which initialises the topics x500/enabled_motors and the x500/command/twist), but when I also initialise the ros_bridge that bridges the gazebo topics with ros2 equivalents I see them. why is that?

### Gazebo lazy topics
Gazebo doesn't show all the topics if nothing is subscribed to it. So its not a valid way of checking if everything is ready to run.

## Gazebo doesn't launch from a python script
### Error: 
```
(venv) paris@hal5000:~/cranfield-ros2-drone-ws/gym-drones$ ./train.sh 
[INFO] Launching ROS 2 sim with command:
bash -c 'source /opt/ros/jazzy/setup.bash && source ~/cranfield-ros2-drone-ws/install/setup.bash && ros2 launch my_drone_sim my_world.launch.py'
[INFO] Waiting for Gazebo topics: ['/x500/camera', '/x500/command/twist']
[ERROR] ros2 launch failed to start.
[STDOUT] [INFO] [launch]: All log files can be found below /home/paris/.ros/log/2025-04-16-11-52-51-631942-hal5000-31245
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [gz-1]: process started with pid [31267]
[gz-1] QObject::moveToThread: Current thread (0x5f58a635e130) is not the object's thread (0x5f58a635a9a0).
[gz-1] Cannot move to target thread (0x5f58a635e130)
[gz-1] 
[gz-1] qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "/home/paris/cranfield-ros2-drone-ws/venv/lib/python3.12/site-packages/cv2/qt/plugins" even though it was found.
[gz-1] This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.
[gz-1] 
[gz-1] Available platform plugins are: xcb, eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, wayland-egl, wayland, wayland-xcomposite-egl, wayland-xcomposite-glx.
[gz-1] 
[gz-1] Stack trace (most recent call last):
[gz-1] #31   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3a9eb9152, in ruby_run_node
[gz-1] #30   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3a9eb4e2b, in 
[gz-1] #29   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa057b49, in rb_vm_exec
[gz-1] #28   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa05462b, in 
[gz-1] #27   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa05013e, in 
[gz-1] #26   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa04d92f, in 
[gz-1] #25   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3a9f8e049, in 
[gz-1] #24   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3a9eb71d6, in rb_protect
[gz-1] #23   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa05c2d9, in rb_yield
[gz-1] #22   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa057b49, in rb_vm_exec
[gz-1] #21   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa05462b, in 
[gz-1] #20   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa05013e, in 
[gz-1] #19   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa04d92f, in 
[gz-1] #18   Object "/usr/lib/x86_64-linux-gnu/ruby/3.2.0/fiddle.so", at 0x7bb3a9c26b13, in 
[gz-1] #17   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x7bb3aa01637b, in rb_nogvl
[gz-1] #16   Object "/usr/lib/x86_64-linux-gnu/ruby/3.2.0/fiddle.so", at 0x7bb3a9c2643b, in 
[gz-1] #15   Object "/lib/x86_64-linux-gnu/libffi.so.8", at 0x7bb3a9cd80bd, in ffi_call
[gz-1] #14   Object "/lib/x86_64-linux-gnu/libffi.so.8", at 0x7bb3a9cd53ee, in 
[gz-1] #13   Object "/lib/x86_64-linux-gnu/libffi.so.8", at 0x7bb3a9cd8b15, in 
[gz-1] #12   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8-gz.so.8.9.0", at 0x7bb3a43afe62, in runGui
[gz-1] #11   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8-gui.so.8", at 0x7bb3a34779ec, in gz::sim::v8::gui::runGui(int&, char**, char const*, char const*, int, char const*, char const*)
[gz-1] #10   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8-gui.so.8", at 0x7bb3a34753c1, in gz::sim::v8::gui::createGui(int&, char**, char const*, char const*, bool, char const*, int, char const*, char const*)
[gz-1] #9    Object "/opt/ros/jazzy/opt/gz_gui_vendor/lib/libgz-gui8.so.8", at 0x7bb3a369003c, in gz::gui::Application::Application(int&, char**, gz::gui::WindowType, char const*)
[gz-1] #8    Object "/lib/x86_64-linux-gnu/libQt5Widgets.so.5", at 0x7bb3a17715b4, in QApplicationPrivate::init()
[gz-1] #7    Object "/lib/x86_64-linux-gnu/libQt5Gui.so.5", at 0x7bb3a033fb9e, in QGuiApplicationPrivate::init()
[gz-1] #6    Object "/lib/x86_64-linux-gnu/libQt5Core.so.5", at 0x7bb3a20deff4, in QCoreApplicationPrivate::init()
[gz-1] #5    Object "/lib/x86_64-linux-gnu/libQt5Gui.so.5", at 0x7bb3a033cc1f, in QGuiApplicationPrivate::createEventDispatcher()
[gz-1] #4    Object "/lib/x86_64-linux-gnu/libQt5Gui.so.5", at 0x7bb3a033c6dc, in QGuiApplicationPrivate::createPlatformIntegration()
[gz-1] #3    Object "/lib/x86_64-linux-gnu/libQt5Core.so.5", at 0x7bb3a1e91103, in QMessageLogger::fatal(char const*, ...) const
[gz-1] #2    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7bb3a9a288fe, in abort
[gz-1] #1    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7bb3a9a4527d, in gsignal
[gz-1] #0    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x7bb3a9a9eb2c, in pthread_kill
[gz-1] Aborted (Signal sent by tkill() 31269 1000) 
```

### Solution
This is coming from your OpenCV installation inside the venv, and it's overriding Gazebo's ability to load the correct system Qt plugins.

To prevent Gazebo from crashing due to Qt plugin conflicts caused by OpenCV in the virtual environment, we sanitized the environment variables (QT_QPA_PLATFORM_PLUGIN_PATH, QT_QPA_FONTDIR, and PYTHONPATH) before launching the simulation from Python. This ensures Gazebo uses the correct system Qt plugins without affecting the rest of the training workflow.

## 