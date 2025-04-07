

## Test the follower simulating the leader

In order to simulate the leader movements, you can run:
1. The velocity command generator using the keyboard, running
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/leader_simulation/cmd_vel
   ```
2. The leader simulator:
    ```bash
   ros2 run nav_follow leader_simulation.py
   ```
4. The follower:
    ```bash
   ros2 launch nav_follow nav_follow.launch.py 
   ```

5. Configure and activate the follower lifecycle nodes
    ```bash
    ros2 lifecycle set /nav_follow configure
    ros2 lifecycle set /nav_follow activate
   ```

