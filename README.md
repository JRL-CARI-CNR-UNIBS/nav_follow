

## Quick Start. Test the follower simulating the leader

Follow the steps below to test the follower by simulating the leader's movements.


**1. Start the velocity command generator using the keyboard:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/leader_simulation/cmd_vel
```

**2.  Launch the leader simulator:**

```bash
ros2 run nav_follow leader_simulation.py
```

**3. Launch the follower:**

```bash
ros2 launch nav_follow nav_follow.launch.py 
```

**4. Configure and activate the follower's lifecycle nodes:**

```bash
ros2 lifecycle set /nav_follow configure
ros2 lifecycle set /nav_follow activate
```

