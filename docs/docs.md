### Programming the robot using this base
- To get started on making the robot, clone this repo and work on the main.py file. By using thisIt is recommended to backup once in a while to avoid any unforeseen errors occuring.
### Controlling your first DC motor.
- First, create a variable with the type being `dc_motor` and in the paramter, please put in the DC port you would like to control. Below is an example.
```python
dc_motor_1 = dc_motor("DC1")
```
- Now, use the function `set_power` of the `dc_motor` class inside of the `run` function in the `manual_stage` to control your first DC motor. Below is an example.
```python
dc_motor_1 = dc_motor("DC1")

class manual:
    
    def __init__(self, mode):
        self.mode = mode
    
    def run(self):
        dc_motor_1.set_power(100)
```
---
