#!/usr/bin/env python3
"""
PMW3389 Dual Sensor Navigation App
Simple road navigation using dual sensor input from Teensy via serial
"""

import pygame
import serial
import sys
import math
import time
import re

# Initialize Pygame
pygame.init()

# Constants
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
ROAD_WIDTH = 200
LANE_WIDTH = 60
FPS = 60

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
DARK_GRAY = (64, 64, 64)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

class Vehicle:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0  # rotation angle in degrees
        self.width = 40
        self.height = 20
        self.speed = 0
        self.max_speed = 5
        
        # Sensor data
        self.sensor1_x = 0
        self.sensor1_y = 0
        self.sensor2_x = 0
        self.sensor2_y = 0
        
        # Movement smoothing
        self.velocity_x = 0
        self.velocity_y = 0
        self.smoothing = 0.8
        
    def update_sensors(self, s1_x, s1_y, s2_x, s2_y):
        """Update sensor readings and calculate movement"""
        self.sensor1_x = s1_x
        self.sensor1_y = s1_y
        self.sensor2_x = s2_x
        self.sensor2_y = s2_y
        
        # Calculate combined movement (average of both sensors)
        avg_dx = (s1_x + s2_x) / 2.0
        avg_dy = (s1_y + s2_y) / 2.0
        
        # Apply smoothing to prevent jittery movement
        self.velocity_x = self.velocity_x * self.smoothing + avg_dx * (1 - self.smoothing)
        self.velocity_y = self.velocity_y * self.smoothing + avg_dy * (1 - self.smoothing)
        
        # Update position with scaling factor
        scale_factor = 0.5
        self.x += self.velocity_x * scale_factor
        self.y += self.velocity_y * scale_factor
        
        # Calculate rotation based on sensor difference
        if s1_x != 0 or s1_y != 0 or s2_x != 0 or s2_y != 0:
            sensor_diff_x = s2_x - s1_x
            if abs(sensor_diff_x) > 2:  # Only rotate if significant difference
                self.angle += sensor_diff_x * 0.1  # Rotation sensitivity
        
        # Keep vehicle on screen
        self.x = max(self.width//2, min(SCREEN_WIDTH - self.width//2, self.x))
        self.y = max(self.height//2, min(SCREEN_HEIGHT - self.height//2, self.y))
        
    def draw(self, screen):
        """Draw the vehicle"""
        # Create vehicle rectangle
        vehicle_surface = pygame.Surface((self.width, self.height))
        vehicle_surface.fill(RED)
        
        # Add direction indicator
        pygame.draw.polygon(vehicle_surface, WHITE, [
            (self.width - 5, self.height//2 - 5),
            (self.width, self.height//2),
            (self.width - 5, self.height//2 + 5)
        ])
        
        # Rotate vehicle
        rotated_surface = pygame.transform.rotate(vehicle_surface, -self.angle)
        rotated_rect = rotated_surface.get_rect(center=(self.x, self.y))
        
        screen.blit(rotated_surface, rotated_rect)
        
        # Draw sensor indicators
        self.draw_sensor_indicators(screen)
        
    def draw_sensor_indicators(self, screen):
        """Draw sensor activity indicators"""
        # Sensor 1 indicator (left side)
        s1_activity = abs(self.sensor1_x) + abs(self.sensor1_y)
        s1_color = GREEN if s1_activity > 0 else GRAY
        s1_size = min(20, 5 + s1_activity)
        pygame.draw.circle(screen, s1_color, (50, 50), int(s1_size))
        
        # Sensor 2 indicator (right side)
        s2_activity = abs(self.sensor2_x) + abs(self.sensor2_y)
        s2_color = BLUE if s2_activity > 0 else GRAY
        s2_size = min(20, 5 + s2_activity)
        pygame.draw.circle(screen, s2_color, (100, 50), int(s2_size))

class Road:
    def __init__(self):
        self.center_x = SCREEN_WIDTH // 2
        self.offset_y = 0
        
    def update(self, vehicle_speed):
        """Update road position based on vehicle movement"""
        self.offset_y += vehicle_speed
        
    def draw(self, screen):
        """Draw the road"""
        # Road background
        road_left = self.center_x - ROAD_WIDTH // 2
        road_right = self.center_x + ROAD_WIDTH // 2
        pygame.draw.rect(screen, DARK_GRAY, (road_left, 0, ROAD_WIDTH, SCREEN_HEIGHT))
        
        # Lane dividers
        lane_positions = [
            self.center_x - LANE_WIDTH,
            self.center_x,
            self.center_x + LANE_WIDTH
        ]
        
        for lane_x in lane_positions:
            for y in range(-50, SCREEN_HEIGHT + 50, 40):
                dash_y = (y + self.offset_y) % SCREEN_HEIGHT
                pygame.draw.rect(screen, YELLOW, (lane_x - 2, dash_y, 4, 20))

class SerialReader:
    def __init__(self, port='/dev/cu.usbmodem151253401', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self.last_data = {'s1_x': 0, 's1_y': 0, 's2_x': 0, 's2_y': 0}
        
    def connect(self):
        """Connect to serial port"""
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to {self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
            
    def read_sensor_data(self):
        """Read and parse sensor data from serial"""
        if not self.serial_connection:
            return self.last_data
            
        try:
            if self.serial_connection.in_waiting > 0:
                line = self.serial_connection.readline().decode('utf-8').strip()
                
                # Parse: "Sensor1 x=5 y=-2 | Sensor2 x=1 y=3"
                match = re.match(r'Sensor1 x=(-?\d+) y=(-?\d+) \| Sensor2 x=(-?\d+) y=(-?\d+)', line)
                if match:
                    self.last_data = {
                        's1_x': int(match.group(1)),
                        's1_y': int(match.group(2)),
                        's2_x': int(match.group(3)),
                        's2_y': int(match.group(4))
                    }
                    
        except Exception as e:
            print(f"Error reading serial data: {e}")
            
        return self.last_data

class NavigationApp:
    def __init__(self):
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("PMW3389 Dual Sensor Navigation")
        self.clock = pygame.time.Clock()
        
        # Initialize components
        self.vehicle = Vehicle(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
        self.road = Road()
        self.serial_reader = SerialReader()
        
        # UI font
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 24)
        
        # Connect to serial
        self.connected = self.serial_reader.connect()
        
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_r:  # Reset vehicle position
                    self.vehicle.x = SCREEN_WIDTH // 2
                    self.vehicle.y = SCREEN_HEIGHT // 2
                    self.vehicle.angle = 0
        return True
        
    def update(self):
        """Update game state"""
        # Read sensor data
        sensor_data = self.serial_reader.read_sensor_data()
        
        # Update vehicle with sensor data
        self.vehicle.update_sensors(
            sensor_data['s1_x'], 
            sensor_data['s1_y'],
            sensor_data['s2_x'], 
            sensor_data['s2_y']
        )
        
        # Update road
        vehicle_speed = (abs(self.vehicle.velocity_x) + abs(self.vehicle.velocity_y)) * 0.1
        self.road.update(vehicle_speed)
        
    def draw(self):
        """Draw everything"""
        # Clear screen
        self.screen.fill(GREEN)  # Grass background
        
        # Draw road
        self.road.draw(self.screen)
        
        # Draw vehicle
        self.vehicle.draw(self.screen)
        
        # Draw UI
        self.draw_ui()
        
        pygame.display.flip()
        
    def draw_ui(self):
        """Draw user interface elements"""
        # Connection status
        status_color = GREEN if self.connected else RED
        status_text = "Connected" if self.connected else "Disconnected"
        status_surface = self.small_font.render(f"Serial: {status_text}", True, status_color)
        self.screen.blit(status_surface, (10, 10))
        
        # Sensor readings
        sensor_data = self.serial_reader.last_data
        sensor_text = f"S1: x={sensor_data['s1_x']:3d} y={sensor_data['s1_y']:3d}  S2: x={sensor_data['s2_x']:3d} y={sensor_data['s2_y']:3d}"
        sensor_surface = self.small_font.render(sensor_text, True, WHITE)
        self.screen.blit(sensor_surface, (10, SCREEN_HEIGHT - 30))
        
        # Instructions
        instructions = [
            "Move your hand over the sensors to navigate",
            "Press R to reset vehicle position",
            "Press ESC to exit"
        ]
        
        for i, instruction in enumerate(instructions):
            text_surface = self.small_font.render(instruction, True, WHITE)
            self.screen.blit(text_surface, (SCREEN_WIDTH - 300, 10 + i * 25))
        
        # Vehicle position
        pos_text = f"Vehicle: x={int(self.vehicle.x)} y={int(self.vehicle.y)} angle={int(self.vehicle.angle)}°"
        pos_surface = self.small_font.render(pos_text, True, WHITE)
        self.screen.blit(pos_surface, (10, SCREEN_HEIGHT - 55))
        
    def run(self):
        """Main game loop"""
        print("Starting PMW3389 Navigation App...")
        print("Make sure your Teensy is connected and sensors are working!")
        
        running = True
        while running:
            running = self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(FPS)
            
        # Cleanup
        if self.serial_reader.serial_connection:
            self.serial_reader.serial_connection.close()
        pygame.quit()

if __name__ == "__main__":
    app = NavigationApp()
    app.run()