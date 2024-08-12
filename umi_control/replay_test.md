```py
conda activate umi
python /home/robot1/umi/universal_manipulation_interface/scripts/xarm_replay_test.py
```

Will show latency Curve if runing in GUI device

Optional Parameter: 

- `--robot_hostname`
  - **Type**: `str`
  - **Default**: `'192.168.202.243'`
  - **Description**: The hostname or IP address of the robot. 
- `--robot_port`
  - **Type**: `int`
  - **Default**: `4243`
  - **Description**: The port number used to connect to the robot.
- `--type`
  - **Type**: `click.Choice(['xarm', 'rtde', 'franka'])`
  - **Default**: `'xarm'`
  - **Description**: The type of robot interface to use. Options are `xarm`, `rtde`, and `franka`.
- `--frequency`
  - **Type**: `float`
  - **Default**: `9`
  - **Description**: The frequency at which data should be processed or sampled.
- `--file_path`
  - **Type**: `str`
  - **Default**: `'/home/robot1/umi/universal_manipulation_interface/data/saved_trajectory/2024-08-06_10-48-37.pkl'`
  - **Description**: The absolute path to the file containing the saved trajectory data.
