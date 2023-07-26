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

## License

MIT
