## move robot with example script
```
robohub@labwork7:~$ cd robohub/jenga/
robohub@labwork7:~/robohub/jenga$ ls
cam_test.py              kinova_venv
Kinova-kortex2_Gen3_G3L  kortex_api-2.8.0.post5-py3-none-any.whl
robohub@labwork7:~/robohub/jenga$ source ./kinova_venv/bin/activate
(kinova_venv) robohub@labwork7:~/robohub/jenga$ cd Kinova-kortex2_Gen3_G3L/api_python/examples/102-Movement_high_level/
(kinova_venv) robohub@labwork7:~/robohub/jenga/Kinova-kortex2_Gen3_G3L/api_python/examples/102-Movement_high_level$ python3 01-move_angular_and_cartesian.py 
Logging as admin on device 192.168.1.10
Moving the arm to a safe position
EVENT : ACTION_START
^CEVENT : ACTION_ABORT
Traceback (most recent call last):
  File "/home/robohub/robohub/jenga/Kinova-kortex2_Gen3_G3L/api_python/examples/102-Movement_high_level/01-move_angular_and_cartesian.py", line 180, in <module>
    exit(main())
         ^^^^^^
  File "/home/robohub/robohub/jenga/Kinova-kortex2_Gen3_G3L/api_python/examples/102-Movement_high_level/01-move_angular_and_cartesian.py", line 170, in main
    success &= example_move_to_home_position(base)
               ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/robohub/robohub/jenga/Kinova-kortex2_Gen3_G3L/api_python/examples/102-Movement_high_level/01-move_angular_and_cartesian.py", line 71, in example_move_to_home_position
    finished = e.wait(TIMEOUT_DURATION)
               ^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/lib/python3.12/threading.py", line 655, in wait
    signaled = self._cond.wait(timeout)
               ^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/lib/python3.12/threading.py", line 359, in wait
    gotit = waiter.acquire(True, timeout)
            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
KeyboardInterrupt

(kinova_venv) robohub@labwork7:~/robohub/jenga/Kinova-kortex2_Gen3_G3L/api_python/examples/102-Movement_high_level$
```

### Get a camera image
```
robohub@labwork7:~$ cd robohub/jenga/
robohub@labwork7:~/robohub/jenga$ ls
cam_test.py              kinova_venv
Kinova-kortex2_Gen3_G3L  kortex_api-2.8.0.post5-py3-none-any.whl
robohub@labwork7:~/robohub/jenga$ source ./kinova_venv/bin/activate
(kinova_venv) robohub@labwork7:~/robohub/jenga$ ./cam_test_gst.py 
```


###  Enable gstreamer use inside venv (once)
```
pip install pycairo  # Required dependency for PyGObject
ln -s /usr/lib/python3/dist-packages/gi ./kinova_venv/lib/python$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages/
python3 -c "import gi; gi.require_version('Gst', '1.0'); from gi.repository import Gst; Gst.init(None); print('GStreamer version:', Gst.version_string())"
```

