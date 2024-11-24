import sys
import socket
import time
import math
from pymavlink import mavutil

class MAVLinkSimulator:
    def __init__(self, port=14550):
        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_address = ('127.0.0.1', port)
        print(f"Initialized UDP socket, sending to {self.target_address}")
        
        # Initialize the MAVLink connection
        # We'll use mavutil to create a connection that we'll use as a message factory
        self.mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        
        # Initial position and movement parameters
        self.lat = 37.7749  # San Francisco latitude
        self.lon = -122.4194  # San Francisco longitude
        self.alt = 100  # Initial altitude in meters
        self.heading = 0
        self.time_boot_ms = 0
        
        # Debug counters
        self.heartbeat_count = 0
        self.position_count = 0
        self.attitude_count = 0
        
    def generate_circular_path(self):
        """Generate a circular flight path"""
        radius = 0.001  # Approximately 100 meters at equator
        angular_speed = 0.1  # radians per second
        
        self.lat += radius * math.cos(angular_speed * self.time_boot_ms / 1000)
        self.lon += radius * math.sin(angular_speed * self.time_boot_ms / 1000)
        self.heading = (angular_speed * self.time_boot_ms / 1000) % (2 * math.pi) * 180 / math.pi
    
    def send_heartbeat(self):
        """Send heartbeat message"""
        try:
            # Pack a heartbeat message
            msg = self.mav.heartbeat_encode(
                mavutil.mavlink.MAV_TYPE_FIXED_WING,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0,  # base_mode
                0,  # custom_mode
                mavutil.mavlink.MAV_STATE_ACTIVE,
                3   # mavlink version
            )
            
            # Get the encoded message buffer
            msg_buf = msg.pack(self.mav)
            
            # Send it
            bytes_sent = self.socket.sendto(msg_buf, self.target_address)
            print(f"Sent HEARTBEAT message ({bytes_sent} bytes)")
            self.heartbeat_count += 1
            
        except Exception as e:
            print(f"Error sending heartbeat message: {e}")
    
    def send_position(self):
        """Send GLOBAL_POSITION_INT message"""
        try:
            # Pack a position message
            msg = self.mav.global_position_int_encode(
                self.time_boot_ms,
                int(self.lat * 1e7),  # Convert to degE7
                int(self.lon * 1e7),
                int(self.alt * 1000),  # Convert to mm
                int(self.alt * 1000),  # altitude above ground
                0,  # velocity x
                0,  # velocity y
                0,  # velocity z
                int(self.heading * 100)  # heading in cdeg
            )
            
            # Get the encoded message buffer
            msg_buf = msg.pack(self.mav)
            
            # Send it
            bytes_sent = self.socket.sendto(msg_buf, self.target_address)
            print(f"Sent POSITION message ({bytes_sent} bytes)")
            print(f"Position: Lat={self.lat:.6f}, Lon={self.lon:.6f}, Alt={self.alt:.1f}, Heading={self.heading:.1f}")
            self.position_count += 1
            
        except Exception as e:
            print(f"Error sending position message: {e}")
    
    def send_attitude(self):
        """Send ATTITUDE message"""
        try:
            # Pack an attitude message
            msg = self.mav.attitude_encode(
                self.time_boot_ms,
                0,  # roll
                0,  # pitch
                math.radians(self.heading),  # yaw
                0,  # roll speed
                0,  # pitch speed
                0   # yaw speed
            )
            
            # Get the encoded message buffer
            msg_buf = msg.pack(self.mav)
            
            # Send it
            bytes_sent = self.socket.sendto(msg_buf, self.target_address)
            print(f"Sent ATTITUDE message ({bytes_sent} bytes)")
            self.attitude_count += 1
            
        except Exception as e:
            print(f"Error sending attitude message: {e}")
    
    def print_stats(self):
        """Print message statistics"""
        print("\nMessage Statistics:")
        print(f"Heartbeat Messages: {self.heartbeat_count}")
        print(f"Position Messages: {self.position_count}")
        print(f"Attitude Messages: {self.attitude_count}")
    
    def run(self):
        """Run the simulator"""
        print(f"Starting MAVLink simulator on port {self.target_address[1]}")
        print(f"Initial position: Lat={self.lat:.6f}, Lon={self.lon:.6f}, Alt={self.alt:.1f}")
        
        try:
            while True:
                # Send heartbeat at 1Hz
                if self.time_boot_ms % 1000 == 0:
                    self.send_heartbeat()
                
                self.generate_circular_path()
                self.send_position()
                self.send_attitude()
                
                self.time_boot_ms += 100  # Increment by 100ms
                
                # Print stats every 5 seconds
                if self.time_boot_ms % 5000 == 0:
                    self.print_stats()
                
                time.sleep(0.1)  # Send at 10Hz
                
        except KeyboardInterrupt:
            print("\nSimulator stopped by user")
            self.print_stats()
        finally:
            self.socket.close()

if __name__ == "__main__":
    # Allow port to be specified as command line argument
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 14550
    simulator = MAVLinkSimulator(port)
    simulator.run()