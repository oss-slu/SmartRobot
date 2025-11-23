# Highest-level Lesson 1: From WASD to Wheel Motion in ROS 2

## Goal of this lesson

By the end of this lesson, you should be able to:

- Drive the robot with **W, A, S, D** using the teleop app.  
- Explain how a **Twist** command (forward + turning velocity) is converted into **specific wheel speeds** and then into **motor (servo) commands**.
- Trace the **ROS 2 control pipeline** at a high level, understanding how the MORPH app creates and feeds ROS2 velocity topic `/diff_drive_base/cmd_vel` into our `diff_drive_controller`, with final exeuction in `ros2_control` hardware, translating to wheel motion.

---

## Task 1: How a Command Reaches the Wheels (Big Picture)

For this lesson, you do **not** need to worry about the web code (using JavaScript, or Flask). Just understand that your **W, A, S, D** actions will send velocity commands to the robot over a local WiFi connection.

When you press **W, A, S, D** in the teleop app, the following happens:

1. The app creates a **Twist** (or `TwistStamped`) message that contains:
   - `linear.x` → how fast the robot should move forward/backward (\(v\)).  
   - `angular.z` → how fast the robot should rotate left/right (\(\omega\)).

2. That Twist is published on a ROS 2 topic, currently named for our model as:

   ```text
   /diff_drive_base/cmd_vel
   ```

3. The **`diff_drive_controller`** ROS2 node subscribes to this topic and uses the Twist to compute left and right wheel speeds.

4. These wheel speeds are sent through **`ros2_control`** to a **hardware plugin** for specific motor control, here being `WaveshareServos`.

5. The hardware plugin converts the wheel velocities into **servo commands** sent over a bus (e.g., serial) to the physical motors.

6. The motors spin at those commanded speeds, and the robot moves.

You should understand each of these stages at a **conceptual** level: where input commands (Twist) feed into a differential drive controller, then servo hardware, and final wheel output.

---

## Task 2: Differential-Drive Kinematics (How v and ω Become Wheel Speeds)

Your robot is a **differential-drive** robot. It mainly has:

- A **left wheel** (or left side wheels).  
- A **right wheel** (or right side wheels).  

Each side can be driven at its own speed.

Let’s define:

- \( v \): forward linear velocity (m/s) = `Twist.linear.x`  
- \( \omega \): angular velocity about the vertical axis (rad/s) = `Twist.angular.z`  
- \( L \): wheel separation (distance between the left and right wheel centers)  
- \( r \): wheel radius

The **linear velocity** of each wheel is:

\[
v_R = v + \frac{\omega L}{2}, \qquad
v_L = v - \frac{\omega L}{2}
\]

The **angular velocity** (what the motor actually executes) is:

\[
\omega_R = \frac{v_R}{r}, \qquad
\omega_L = \frac{v_L}{r}
\]

Special cases:

- **Straight forward**: \( \omega = 0 \Rightarrow v_R = v_L = v \)  
- **Straight backward**: \( \omega = 0, v < 0 \)  
- **Spin in place left**: \( v = 0, \omega > 0 \Rightarrow v_R = +\omega L/2, v_L = -\omega L/2 \)  
- **Spin in place right**: \( v = 0, \omega < 0 \Rightarrow v_R = -\omega L/2, v_L = +\omega L/2 \)

The ROS 2 **`diff_drive_controller`** is essentially implementing these equations for you internally.

The **key idea** here is that a high-level Twist (\(v, \omega\)) is automatically translated into **specific wheel speeds** (\(\omega_L, \omega_R\)) using your robot’s geometry (\(L\) and \(r\)).

---

## Task 3: Where Geometry Lives in Your ROS 2 Package

To do the math above correctly, the controller needs to know your robot’s geometry: **wheel radius** and **wheel separation**.

These values appear in two main places:

### 3.1 URDF / Xacro: the robot’s physical description

In a file like:

```text
ros2_ws/src/waveshare_servos/description/urdf/example.urdf.xacro
```

you might see properties such as:

```xml
<xacro:property name="wheel_radius" value="0.050"/>      <!-- 5 cm -->
<xacro:property name="wheel_sep"    value="0.280"/>      <!-- distance between wheel centers -->
```

The URDF/Xacro defines:

- What the robot **looks like**.  
- How big the wheels are (`wheel_radius`).  
- How far apart the wheels are (`wheel_sep`).  

These values describe the **real robot**, and should match what you measure with a ruler.

### 3.2 Controller YAML: telling diff_drive_controller the geometry

In a file like:

```text
ros2_ws/src/waveshare_servos/bringup/config/example_controllers.yaml
```

you’ll see the diff-drive controller configuration:

```yaml
diff_drive_base:
  ros__parameters:
    left_wheel_names:  ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    # Geometry (maching the URDF)
    wheel_separation: 0.280     # meters
    wheel_radius:     0.050     # meters
```

This tells **`diff_drive_controller`**:

- Which joints are the **left** and **right** wheels.  
- The **wheel separation** and **radius** to use in its kinematic calculations.

This creates a **connection** across files, where the numbers in the controller YAML must match the URDF/Xacro. If they don’t, the mapping from (\(v, \omega\)) to wheel speeds will be wrong, and the robot will not drive as expected.

---

## Task 4: ros2_control and the Hardware Plugin: From Wheel Speeds to Motor Commands

Once `diff_drive_controller` computes desired wheel velocities, those velocities are passed to the **ros2_control** system and then down to a **hardware plugin** that talks to the motors.

### 4.1 ros2_control: mapping joints to hardware

In a file like:

```text
ros2_ws/src/waveshare_servos/description/ros2_control/example.ros2_control.xacro
```

you’ll see something like:

```xml
<ros2_control name="example_ws_ros2_control" type="system">
  <hardware>
    <plugin>waveshare_servos/WaveshareServos</plugin>
  </hardware>

  <!-- LEFT wheel -->
  <joint name="left_wheel_joint">
    <param name="id">2</param>
    <param name="type">vel</param>
    <command_interface name="velocity">
      <param name="min">-9.2</param>
      <param name="max"> 9.2</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- RIGHT wheel -->
  <joint name="right_wheel_joint">
    <param name="id">1</param>
    <param name="type">vel</param>
    <param name="invert">true</param>
    <command_interface name="velocity">
      <param name="min">-9.2</param>
      <param name="max"> 9.2</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

This tells ROS 2:

- The `left_wheel_joint` uses a hardware motor with ID 2.  
- The `right_wheel_joint` uses a hardware motor with ID 1.  
- Both joints are controlled in **velocity** mode.  
- There is a specific plugin (`WaveshareServos`) that knows how to talk to these motors.

### 4.2 Hardware plugin: converting wheel velocities into servo-specific units

In the C++ hardware plugin (e.g. `waveshare_servos.cpp`), the `write()` function takes the **desired joint velocities** from `diff_drive_controller` and converts them into servo commands.

Conceptually, it looks like this:

```cpp
// Simplified example pattern
for (each wheel joint) {
    double joint_velocity = commanded_velocity_from_controller;
    double servo_velocity = SCALE_FACTOR * joint_velocity;  // convert rad/s to servo units
    send_to_servo(servo_id, servo_velocity);
}
```

The key steps for this setup is to:

1. **Read joint velocities** (wheel speeds) produced by `diff_drive_controller`.  
2. **Convert** those from rad/s into whatever units the servo expects (e.g., encoder steps per second).  
3. **Send** those values over the communication bus to each motor using its ID.

Connections to understand are:

1. At a **High-level** the MORPH app sends a desired forward and turning speed (a Twist).  
2. At a **Mid-level** the `diff_drive_controller` turns (\(v, \omega\)) into left/right wheel velocities based on `wheel_radius` and `wheel_separation`.  
3. At a **Low-level** the hardware plugin converts those wheel velocities into **motor-specific commands** (like “motor 1: 200 steps/s, motor 2: –200 steps/s”).

This is how “press W” becomes **actual motor currents and wheel motion**.

---

## Task 5: Observing Hands-on Commands in ROS 2

### 5.1 Watch the command topic while driving

1. Make sure your robot and bringup launch are running (TODO DESCRIBE THIS).  
2. On the robot, list topics:

   ```bash
   ros2 topic list
   ```

    Look for a command topic:

   ```text
   /diff_drive_base/cmd_vel
   ```

3. Echo the topic:

   ```bash
   ros2 topic echo /diff_drive_base/cmd_vel
   ```

4. In the teleop app, press:

   - **W** (forward): `linear.x` should become positive, `angular.z` ~ 0.  
   - **S** (backward): `linear.x` should become negative.  
   - **A** (turn left): `angular.z` should become positive, `linear.x` ~ 0.  
   - **D** (turn right): `angular.z` should become negative, `linear.x` ~ 0.  
   - **Stop**: both should return to near zero.

You are now **seeing** the high-level Twist commands that eventually become wheel speeds.

### 5.2 Relate the numbers to motion

Use approximate geometry from your config, for example:

- `wheel_separation` \( L = 0.280 \) m  
- `wheel_radius` \( r = 0.050 \) m  

Try this thought experiment (or real experiment):

1. Suppose W sets `linear.x ≈ 0.5` m/s and `angular.z = 0`.  
   - Compute:  
     \( v_R = v_L = 0.5 \) m/s.  
     \( \omega_R = \omega_L = v / r = 0.5 / 0.05 = 10 \) rad/s.
   - The hardware plugin will send motor commands corresponding to \( \omega_R \approx \omega_L \approx 10 \) rad/s.

2. Suppose A sets `linear.x = 0`, `angular.z ≈ 1.0` rad/s.  
   - Compute:  
     \( v_R = +L/2 = 0.140 \) m/s, \( v_L = -0.140 \) m/s.  
     The right wheel moves forward, the left wheel moves backward, so the robot **spins in place**.

These numbers are exactly what `diff_drive_controller` and your hardware plugin work with behind the scenes.

---

## Task 6: Summary: From WASD to Motor Control

You should now be able to summarize the full story like this:

“When I press W in the teleop app, it sends a `Twist` into ROS 2 on the drive command topic (with a positive `linear.x` and zero `angular.z`). The `diff_drive_controller` uses the robot’s `wheel_separation` and `wheel_radius` to convert that desired forward velocity into left and right wheel velocities. Those wheel velocities are passed through `ros2_control` to the hardware plugin, which converts them into servo-specific commands for each motor. The motors then spin at those commanded speeds, and the robot moves straight forward.”

If you can:

- Drive with W/A/S/D,  
- Observe the corresponding Twist messages in ROS 2, and  
- Explain how those messages become wheel and motor commands,

then you’ve successfully completed **Lesson 1: Learning How to Drive**