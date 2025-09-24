# PMW3389 Dual Sensor Navigation App

A simple Python application that visualizes movement from dual PMW3389 sensors as navigation on a straight road.

## Features

- **Real-time sensor visualization**: See your sensor readings as a red vehicle moving on a road
- **Dual sensor input**: Uses both sensors to calculate movement and rotation
- **Smooth movement**: Applies smoothing to prevent jittery motion
- **Visual feedback**: Colored indicators show sensor activity levels
- **Simple road interface**: Navigate along a multi-lane road with lane markers

## Setup

### 1. Install Python Dependencies

```bash
pip install -r requirements.txt
```

### 2. Hardware Setup

Make sure your Teensy 4.0 is connected with both PMW3389 sensors:

```
Teensy Pin 9  → Sensor 1 NCS
Teensy Pin 10 → Sensor 2 NCS
3.3V          → Both sensors' RST pins
MOSI, MISO, SCK, VCC, GND → Shared between sensors
```

### 3. Upload Teensy Code

Upload the `main.cpp` code to your Teensy using PlatformIO.

## Usage

### Run the Application

```bash
python navigation_app.py
```

### Controls

- **Move your hand over the sensors**: The red vehicle will move accordingly
- **Both sensors together**: Vehicle moves forward/backward and left/right
- **Different sensor movements**: Vehicle rotates
- **Press R**: Reset vehicle to center position
- **Press ESC**: Exit the application

### Visual Elements

- **Red vehicle**: Your position and orientation
- **Green circle (top-left)**: Sensor 1 activity indicator
- **Blue circle (top-left)**: Sensor 2 activity indicator  
- **Road with yellow lane markers**: Navigation environment
- **Status indicators**: Connection status and sensor readings

## How It Works

1. **Sensor Reading**: Reads serial data from Teensy in format:
   ```
   Sensor1 x=5 y=-2 | Sensor2 x=1 y=3
   ```

2. **Movement Calculation**: 
   - Average of both sensors determines forward/backward and left/right movement
   - Difference between sensors determines rotation
   - Smoothing applied to prevent jittery movement

3. **Visualization**:
   - Vehicle position updates based on sensor input
   - Road scrolls to simulate forward motion
   - Sensor activity indicators show real-time readings

## Troubleshooting

### Serial Connection Issues
- Check that your Teensy is connected via USB
- Verify the correct serial port in the code (default: `/dev/cu.usbmodem151253401`)
- Ensure no other programs are using the serial port

### No Movement
- Check that both sensors are properly connected and powered
- Verify sensor readings in the bottom status bar
- Try moving your hand closer to the sensors

### Jittery Movement
- Increase the smoothing factor in the `Vehicle` class
- Check for electrical interference or loose connections
- Ensure stable power supply to sensors

## Customization

### Adjust Movement Sensitivity
In `navigation_app.py`, modify these values in the `Vehicle.update_sensors()` method:
- `scale_factor`: Controls movement speed (default: 0.5)
- `smoothing`: Controls movement smoothing (default: 0.8)
- Rotation sensitivity: Modify the `* 0.1` factor for rotation

### Change Serial Port
Update the `SerialReader.__init__()` method with your serial port:
```python
def __init__(self, port='/dev/cu.usbmodem151253401', baudrate=9600):
```

### Visual Customization
- Colors defined at the top of the file
- Screen dimensions: `SCREEN_WIDTH` and `SCREEN_HEIGHT`
- Road and vehicle sizes: `ROAD_WIDTH`, `vehicle.width`, etc.

## Next Steps

This basic navigation app can be extended with:
- More complex road layouts (curves, intersections)
- Obstacles and collision detection
- Speed control and acceleration
- Multiple vehicles or multiplayer support
- Data logging and analysis features