# ROS2 Learning Journey

## Projects

### py_pubsub - Basic Publisher/Subscriber
**Description:**
My first ROS2 package. 
`talker` publishes messages to `chatter` topic on interval.
`listener` logs messages from `chatter` topic.

**Run it:**
```bash
# Terminal 1
ros2 run py_pubsub talker

# Terminal 2
ros2 run py_pubsub listener
```