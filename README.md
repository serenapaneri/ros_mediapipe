# Instruction to install cv_bridge for python3

The dockerfile contains the instruction needed to be able to use the cv_bridge in the python3 environment.
In particular the lines needed to be execute in the terminal are between lines 70 and 80.

### In your python script
After installing the cv_bridge for python3, following the abovementioned instruction, you also need the following lines of code in your python scripts:
```
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/home/spot/cv_bridge_ws/install/lib/python3/dist-packages')
```


