# ROS2 Learning Journey

ROS2 Kilted workspace for learning.

## Prerequisites

- ROS2 Kilted: https://docs.ros.org/en/kilted/Installation.html
- colcon: `sudo apt install python3-colcon-common-extensions`

## Setup

```bash
git clone <repo-url> ros2-learning
cd ros2-learning
./setup.sh
```

After setup completes, add the printed alias to your `~/.bashrc`:
```bash
alias ros2-learning="cd ~/Documents/ros2-learning && source install/setup.bash"
```

Then reload: `source ~/.bashrc`

## Development

In each new terminal, source the workspace:
```bash
ros2-learning
```

Common commands:
```bash
make build   # Rebuild packages
make clean   # Remove build artifacts
```

## Projects

### py_pubsub - Basic Publisher/Subscriber
My first ROS2 package.
`talker` publishes messages to `chatter` topic on interval.
`listener` logs messages from `chatter` topic.

```bash
ros2 run py_pubsub talker
ros2 run py_pubsub listener
```

### speech_recognition - Local Speech Recognition
Offline speech-to-text using Whisper (via faster-whisper) with Silero VAD (Voice Activity Detection).

Models are downloaded automatically on first run to `models/` in the repo root.

Publishes `std_msgs/String` to `/speech` (configurable)

```bash
ros2 run speech_recognition speech_node
ros2 run speech_recognition speech_node --ros-args -p model:=base
ros2 topic echo /speech
```

#### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| model | small | Whisper model size (tiny, base, small, medium, large) |
| sample_rate | 16000 | Audio sample rate in Hz (must be 16000 for VAD) |
| device_index | -1 | Audio device index (-1 for system default) |
| topic_name | speech | Topic name for publishing transcriptions |
| vad_threshold | 0.5 | VAD confidence threshold (0.0-1.0) |
| vad_silence_ms | 500 | Silence duration in ms to end utterance |

#### Pre-download Models (Optional)
```bash
make download-models
```

#### Whisper Models
| Model | Download | RAM | Speed | Accuracy |
|-------|----------|-----|-------|----------|
| tiny | ~75 MB | ~1 GB | Fastest | Low |
| base | ~140 MB | ~1.5 GB | Fast | Medium |
| small | ~460 MB | ~2.5 GB | Medium | Good |
| medium | ~1.5 GB | ~5 GB | Slow | High |
