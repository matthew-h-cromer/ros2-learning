# ROS2 Learning Project

ROS2 Kilted workspace for learning.

## Commands
- Build: `colcon build --symlink-install`
- Source: `source install/setup.bash`
- New package: `ros2 pkg create --build-type ament_python <name>`
- Run node: `ros2 run <package> <executable>`

## Packages
- `pubsub` - Basic publisher/subscriber example
- `speech_recognition` - Local speech-to-text using Whisper

## speech_recognition Parameters
- `model` (default: small) - Whisper model size
- `sample_rate` (default: 16000) - Audio sample rate
- `device_index` (default: -1) - Audio device (-1 = system default)
- `topic_name` (default: speech) - Output topic name
- `vad_threshold` (default: 0.5) - VAD confidence threshold
- `vad_silence_ms` (default: 500) - Silence duration to end utterance

## Style
- Python: snake_case, type hints encouraged
- Use rclpy logging: `self.get_logger().info(...)`
