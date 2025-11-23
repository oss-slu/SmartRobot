# Middle-level Lesson 1: Learning How to Drive

**Goal of this lesson**  
By the end, you should be able to:

- Drive the robot using **W, A, S, D**.
- Explain how its **left** and **right** wheels move to make it go straight or turn.
- Understand that our code sends **speeds** to the robot, not just “go” or “turn”.

---

## Task 1: How Your Robot Steers

Your robot is a **differential-drive** robot. That means it mainly uses:

- A **left wheel** (or left side wheels).
- A **right wheel** (or right side wheels).

Each side can be driven at its own speed.

Let’s call:

- \( velocity_left \) = speed of the **left** wheel  
- \( velocity_right \) = speed of the **right** wheel  

Then:

- **Go straight forward:** \( velocity_left = velocity_right > 0 \)  
- **Go straight backward:** \( velocity_left = velocity_right < 0 \)  
- **Turn left in place:** \( velocity_left = -velocity_right \), with \( velocity_right > 0 \)  
- **Turn right in place:** \( velocity_right = -velocity_left \), with \( velocity_left > 0 \)

You don’t have to do heavy math yet; just remember:

- Straight = both wheels same speed, same direction.  
- Spin = wheels same speed, opposite direction.

---

## Task 2: Open the Driving Interface

Open the **robot driving app**.

You’ll see:

- A status area that shows if the robot is **connected**.
- Direction buttons like **Up, Down, Left, Right, Stop**.
- Possibly a **speed slider**.

Make sure the robot is on the floor with some space.

---

## Task 3: Try Driving with Buttons

Practice with the on-screen buttons first:

1. Press **Up**.  
   - What happens to the robot?  
   - What do you think \( velocity_left \) and \( velocity_right \) are doing?  
   Expected: both wheels forward → \( velocity_left \approx velocity_right > 0 \)

2. Press **Down**.  
   - Both backward → \( velocity_left \approx velocity_right < 0 \)

3. Press **Left**.  
   - It rotates left. Likely one wheel goes forward and the other backward:  
     - \( velocity_left < 0 \), \( velocity_right > 0 \)

4. Press **Right**.  
   - Opposite pattern.

5. Press **Stop**.  
   - Both \( velocity_left \) and \( velocity_right \) are near zero.

Write down your guesses:

- Up: \( velocity_left = \_\_\_ \), \( velocity_right = \_\_\_ \)  
- Down: \( velocity_left = \_\_\_ \), \( velocity_right = \_\_\_ \)  
- Left: \( velocity_left = \_\_\_ \), \( velocity_right = \_\_\_ \)  
- Right: \( velocity_left = \_\_\_ \), \( velocity_right = \_\_\_ \)

---

## Task 4: What the Code Is Really Sending

Our robot doesn’t receive “go up” or “go left” as text. It receives **numbers**: forward speed and turning speed.

In ROS 2, we usually send a **Twist** message with:

- `linear.x` → how fast to go **forward/backward** (meters per second).  
- `angular.z` → how fast to **turn** left/right (radians per second).

Rough idea:

- **Forward**: `linear.x > 0`, `angular.z = 0`  
- **Backward**: `linear.x < 0`, `angular.z = 0`  
- **Turn left in place**: `linear.x = 0`, `angular.z > 0`  
- **Turn right in place**: `linear.x = 0`, `angular.z < 0`

The app does this mapping for you when you click the buttons or press W/A/S/D.

---

## Task 5: From W, A, S, D to Motion

In our app, the keyboard is set up like a game:

- **W** → “move up” → something like `linear.x = +0.5`, `angular.z = 0.0`  
- **S** → “move down” → `linear.x = -0.5`, `angular.z = 0.0`  
- **A** → “move left” → `linear.x = 0.0`, `angular.z = +0.5`  
- **D** → “move right” → `linear.x = 0.0`, `angular.z = -0.5`  
- (Space or Stop) → `linear.x = 0.0`, `angular.z = 0.0`

The input keys choose the **forward speed** and **turning speed** that we send to the robot.

---

## Task 6: Try It: Keyboard Driving

Now try driving only with the keyboard:

1. Press **W** for 1 second, then release.  
   - How far did the robot move?

2. Press **S** for 1 second.  
   - It should move about the same distance backward.

3. Press **A** for 1 second.  
   - It should spin left.

4. Press **D** for 1 second.  
   - It should spin right.

5. Press **Stop** or space to halt.

If your app has a **speed slider**, try this experiment:

- Set the speed low, use **W**, and see how far it goes in 2 seconds.  
- Set the speed high, use **W**, and see how much farther it goes in 2 seconds.

Write what you notice.

---

## Task 7: What You Should Understand

By the end of this lesson, you should be able to explain in your own words:

1. **How the wheels move:**
   - “To go straight, left and right wheels move together.”  
   - “To turn, the wheels move differently from each other.”

2. **How the robot is controlled:**
   - “When I press W/A/S/D, the app sends forward and turning speeds (a Twist) to the robot.”

If you can say that clearly *and* drive around a simple course, you’ve completed **Lesson 1: Learning How to Drive**
