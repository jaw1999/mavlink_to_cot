# MAVLink to CoT Converter

A Python-based web application that converts MAVLink telemetry data to Cursor on Target (CoT) format for integration with TAK systems. 

## Features

- Real-time conversion of MAVLink telemetry to CoT
- Web-based configuration interface
- Support for both unicast and multicast CoT transmission
- Live debugging and monitoring
- Position, altitude, and heading tracking
- Configurable aircraft identification
- MAVLink simulator for testing

## Requirements

- Python 3.6 or higher
- Virtual environment (recommended)

## Installation

1. Clone the repository:
```bash

cd mavlink-to-cot
```

2. Create and activate a virtual environment:

For Linux/MacOS:
```bash
python3 -m venv venv
source venv/bin/activate
```

For Windows:
```cmd
python -m venv venv
venv\Scripts\activate
```

3. Install required packages:
```bash
pip install -r requirements.txt
```

## Usage

1. Start the converter:
```bash
python mavlink_to_cot.py
```
The web interface will be available at `http://localhost:8080`

2. For testing, start the simulator in another terminal:
```bash
python mavlink_simulator.py 14550
```

### Configuration Options

#### MAVLink Settings
- **Port**: UDP port for MAVLink data (default: 14550)
- **Aircraft Name**: Identifier for the aircraft in TAK

#### CoT Settings
- **IP Address**: Destination IP for CoT data
- **Port**: Destination port for CoT data
- **Multicast**: Toggle between unicast and multicast transmission

## Testing

The included MAVLink simulator (`mavlink_simulator.py`) generates test data that includes:
- Position (latitude, longitude)
- Altitude
- Heading
- Circular flight pattern

## Debug Information

The debug panel provides:
- Message counters for MAVLink and CoT
- Latest position data
- Real-time log messages
- Connection status

## File Structure

```
mavlink-to-cot/
├── mavlink_to_cot.py     # Main converter application
├── mavlink_simulator.py   # Test data simulator
├── requirements.txt      # Python dependencies
├── README.md            # This file
└── venv/               # Virtual environment directory
```

## Network Configuration

### For Unicast
1. Set CoT IP to your TAK server IP
2. Set CoT port to match your TAK server configuration (typically 8087)
3. Uncheck "Use Multicast"

### For Multicast
1. Use default multicast address (239.2.3.1) or your network's multicast group
2. Configure port according to your network setup
3. Check "Use Multicast"

## Contributing

Contributions are welcome! Please feel free to submit pull requests.

## Troubleshooting

### Common Issues

1. No MAVLink Messages
   - Verify simulator is running
   - Check MAVLink port configuration
   - Ensure no firewall blocking

2. No CoT in TAK
   - Verify IP and port settings
   - Check network connectivity
   - Confirm TAK server configuration

### Debug Steps

1. Start the converter
2. Open the Debug Information tab
3. Monitor message counters and logs
4. Check for any error messages

