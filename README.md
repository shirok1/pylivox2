# PyLivox2

Hackable CLI (and library) for controlling Livox LiDAR sensors, in pure Python.

## Requirements

- Python 3.10+

Following Python packages are required, and can be installed with `pip install -r requirements.txt`:

- `crc`
- `loguru`
- `numpy`

## Usage

```shell
# git clone ... && cd ...
python3 -m pylivox2 --help
# specify IP address
python3 -m pylivox2 -d {action} # discover
python3 -m pylivox2 --ip -- {action} # known IP, -- for 192.168.1.100
```

## ROS2

This repo also works as a ROS2 node.

```shell
# clone to src/livox2_ctrl
git clone https://github.com/shirok1/pylivox2.git src/livox2_ctrl

# install in workspace root
colcon build --symlink-install

# run
# you may want to `export ROS_LOCALHOST_ONLY=1`
ros2 run livox2_ctrl daemon
```

## License

MIT
