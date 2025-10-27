# **SMART: SLU Mobile Autonomous Robotics Toolkit**
## A learning robot kit.

## Getting Started (ROS2 Jazzy)
### Clone the repository
```bash
git clone https://github.com/oss-slu/SmartRobot.git
cd SmartRobot
```
### Build the ROS2 components
```bash
git clone https://github.com/oss-slu/SmartRobot.git
cd ros2_ws
*TODO, add rosdep step?*
colcon build
```

## Getting Started (Desktop client)
### Create and activate a virtual environment
```bash
python -m venv venv
source venv/bin/activate   # macOS/Linux
venv\Scripts\activate      # Windows
```
### Install dependencies
```bash
pip install -r requirements.txt
```
### Run the application
```bash
python app.py
```
### Open in browser
```bash
Visit: http://127.0.0.1:5000/
```

## Getting Driving (Desktop client)
### TODO - Talk about setting up the controller on the client
To send the robot forward 2 m/s (for testing purposes), utilize this twist message publisher in a new (sourced) terminal
```bash
ros2 topic pub -r 10 /diff_drive_base/cmd_vel geometry_msgs/TwistStamped \ "{header: {frame_id: base_link}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```


## Support




## Purchase
- Please visit the following page to purchase our products:
….
- Business customers please contact us through the following email address:
…


## Copyright


## About
- SMART is an open-source electronics platform.
….
- Our services include:
….
- Our code and circuit are open source. You can obtain the details and the latest information through visiting the following web site:

....
