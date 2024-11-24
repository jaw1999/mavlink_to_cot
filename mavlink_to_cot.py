import sys
import socket
import threading
from pymavlink import mavutil
from datetime import datetime, timedelta
import xml.etree.ElementTree as ET
from flask import Flask, render_template_string, request, jsonify
import time
import struct
import json
import queue

class MAVLinkToCoT:
    def __init__(self):
        self.mavlink_connection = None
        self.cot_socket = None
        self.aircraft_name = "DEFAULT_UAV"
        self.running = False
        self.cot_ip = "239.2.3.1"
        self.cot_port = 6969
        self.use_multicast = True
        
        # Debug counters and latest data
        self.mavlink_msg_count = 0
        self.cot_msg_count = 0
        self.latest_position = None
        self.debug_queue = queue.Queue(maxsize=100)
        
    def get_debug_info(self):
        return {
            "mavlink_count": self.mavlink_msg_count,
            "cot_count": self.cot_msg_count,
            "latest_position": self.latest_position,
            "running": self.running
        }

    def add_debug_message(self, message):
        try:
            self.debug_queue.put(message, block=False)
        except queue.Full:
            try:
                # Remove oldest message if queue is full
                self.debug_queue.get_nowait()
                self.debug_queue.put(message, block=False)
            except:
                pass

    def get_debug_messages(self):
        messages = []
        while not self.debug_queue.empty():
            try:
                messages.append(self.debug_queue.get_nowait())
            except queue.Empty:
                break
        return messages

    def generate_cot_xml(self, lat, lon, alt, heading):
        """Generate CoT XML message for UAV position"""
        root = ET.Element("event")
        root.set("version", "2.0")
        root.set("uid", self.aircraft_name)
        root.set("type", "a-f-A-M-F-Q")  # Friendly UAV
        
        current_time = datetime.utcnow()
        stale_time = current_time + timedelta(minutes=1)
        
        root.set("time", current_time.strftime("%Y-%m-%dT%H:%M:%SZ"))
        root.set("start", current_time.strftime("%Y-%m-%dT%H:%M:%SZ"))
        root.set("stale", stale_time.strftime("%Y-%m-%dT%H:%M:%SZ"))
        root.set("how", "m-g")  # GPS

        # Add point data
        point = ET.SubElement(root, "point")
        point.set("lat", f"{lat:.6f}")
        point.set("lon", f"{lon:.6f}")
        point.set("hae", f"{alt:.1f}")
        point.set("ce", "10.0")
        point.set("le", "3.0")

        # Add detail data with enhanced track info
        detail = ET.SubElement(root, "detail")
        
        # Add contact info to set the callsign/name
        contact = ET.SubElement(detail, "contact")
        contact.set("callsign", self.aircraft_name)
        
        # Add track info including speed and heading
        track = ET.SubElement(detail, "track")
        track.set("course", f"{heading:.1f}")
        track.set("speed", "0.00")
        
        # Add icon orientation using the __dir field
        usericon = ET.SubElement(detail, "__dir")
        usericon.text = f"{heading:.1f}"
        
        # Add remarks for additional info
        remarks = ET.SubElement(detail, "remarks")
        remarks.text = f"{self.aircraft_name} - Altitude: {alt:.1f}m, Heading: {heading:.1f}°"

        return ET.tostring(root)

    def start_mavlink_connection(self, connection_string):
        try:
            # For UDP connections, we need to explicitly specify the input and output
            input_port = int(connection_string.split(':')[-1])
            self.mavlink_connection = mavutil.mavlink_connection(
                'udp:0.0.0.0:' + str(input_port),
                input=True,
                source_system=255  # Use 255 to indicate we're a ground station
            )
            self.mavlink_connection.wait_heartbeat(timeout=5.0)
            self.add_debug_message(f"MAVLink connection established and heartbeat received")
            return True
        except Exception as e:
            self.add_debug_message(f"Error connecting to MAVLink: {e}")
            return False

    def setup_cot_socket(self):
        try:
            self.cot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if self.use_multicast:
                self.cot_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
            self.add_debug_message(f"CoT socket setup complete - {'Multicast' if self.use_multicast else 'Unicast'}")
            return True
        except Exception as e:
            self.add_debug_message(f"Error setting up CoT socket: {e}")
            return False

    def process_mavlink_messages(self):
        self.add_debug_message(f"Starting MAVLink processing, sending CoT to {self.cot_ip}:{self.cot_port}")
        
        # Explicitly define which messages we want to process
        REQUIRED_MESSAGES = {
            'GLOBAL_POSITION_INT': {
                'received': False,
                'rate': 0,
                'last_time': 0,
                'required_fields': ['lat', 'lon', 'alt', 'hdg']
            }
        }
        
        while self.running:
            try:
                # Only listen for the messages we need
                msg = self.mavlink_connection.recv_match(
                    type=['GLOBAL_POSITION_INT'],
                    blocking=True,
                    timeout=1.0
                )
                
                if msg is not None:
                    msg_type = msg.get_type()
                    self.mavlink_msg_count += 1
                    
                    # Process position data
                    if msg_type == 'GLOBAL_POSITION_INT':
                        # Check if message has all required fields
                        has_all_fields = all(hasattr(msg, field) 
                                           for field in REQUIRED_MESSAGES[msg_type]['required_fields'])
                        
                        if has_all_fields:
                            # Convert and validate data
                            try:
                                lat = float(msg.lat) / 1e7  # Convert from degE7
                                lon = float(msg.lon) / 1e7
                                alt = float(msg.alt) / 1000.0  # Convert from mm to meters
                                heading = float(msg.hdg) / 100.0 if msg.hdg != 0 else 0  # Convert from cdeg
                                
                                # Basic sanity checks
                                if (-90 <= lat <= 90 and 
                                    -180 <= lon <= 180 and 
                                    -1000 <= alt <= 60000 and  # reasonable altitude range in meters
                                    0 <= heading <= 360):
                                    
                                    # Update message stats
                                    current_time = time.time()
                                    if REQUIRED_MESSAGES[msg_type]['received']:
                                        time_diff = current_time - REQUIRED_MESSAGES[msg_type]['last_time']
                                        if time_diff > 0:
                                            REQUIRED_MESSAGES[msg_type]['rate'] = 1.0 / time_diff
                                    
                                    REQUIRED_MESSAGES[msg_type]['received'] = True
                                    REQUIRED_MESSAGES[msg_type]['last_time'] = current_time
                                    
                                    # Update latest position
                                    self.latest_position = {
                                        "lat": lat,
                                        "lon": lon,
                                        "alt": alt,
                                        "heading": heading,
                                        "timestamp": datetime.utcnow().strftime("%H:%M:%S")
                                    }
                                    
                                    # Generate and send CoT
                                    try:
                                        cot_xml = self.generate_cot_xml(lat, lon, alt, heading)
                                        self.cot_socket.sendto(cot_xml, (self.cot_ip, self.cot_port))
                                        self.cot_msg_count += 1
                                        
                                        # Log with rate information
                                        self.add_debug_message(
                                            f"Sent CoT #{self.cot_msg_count} - "
                                            f"Lat: {lat:.6f}, Lon: {lon:.6f}, "
                                            f"Alt: {alt:.1f}m, Heading: {heading:.1f}° "
                                            f"(Rate: {REQUIRED_MESSAGES[msg_type]['rate']:.1f} Hz)"
                                        )
                                    except Exception as e:
                                        self.add_debug_message(f"Error sending CoT message: {e}")
                                else:
                                    self.add_debug_message(f"Invalid data received: Lat={lat}, Lon={lon}, Alt={alt}, Heading={heading}")
                            except (ValueError, AttributeError) as e:
                                self.add_debug_message(f"Error converting position data: {e}")
                        else:
                            missing_fields = [field for field in REQUIRED_MESSAGES[msg_type]['required_fields'] 
                                           if not hasattr(msg, field)]
                            self.add_debug_message(f"Missing required fields: {missing_fields}")
                    
            except Exception as e:
                self.add_debug_message(f"Error processing MAVLink message: {e}")
                time.sleep(1)

    def start_conversion(self, mavlink_port, aircraft_name, cot_ip, cot_port, use_multicast):
        if self.running:
            return "Already running"
        
        self.aircraft_name = aircraft_name
        self.cot_ip = cot_ip
        self.cot_port = int(cot_port)
        self.use_multicast = use_multicast
        
        connection_string = f'udpin:127.0.0.1:{mavlink_port}'
        
        self.add_debug_message(f"\nStarting conversion with settings:")
        self.add_debug_message(f"MAVLink Port: {mavlink_port}")
        self.add_debug_message(f"Aircraft Name: {aircraft_name}")
        self.add_debug_message(f"CoT IP: {cot_ip}")
        self.add_debug_message(f"CoT Port: {cot_port}")
        self.add_debug_message(f"Using Multicast: {use_multicast}\n")
        
        if not self.start_mavlink_connection(connection_string):
            return "Failed to connect to MAVLink"
        
        if not self.setup_cot_socket():
            return "Failed to setup CoT socket"
        
        self.running = True
        self.conversion_thread = threading.Thread(target=self.process_mavlink_messages)
        self.conversion_thread.start()
        
        return "Conversion started successfully"

    def stop_conversion(self):
        if not self.running:
            return "Not running"
            
        self.running = False
        if self.conversion_thread:
            self.conversion_thread.join()
        if self.mavlink_connection:
            self.mavlink_connection.close()
        if self.cot_socket:
            self.cot_socket.close()
        return "Conversion stopped"

app = Flask(__name__)
converter = MAVLinkToCoT()

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>MAVLink to CoT Converter</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #1a1a1a;
            color: #e0e0e0;
            margin: 0;
            padding: 0;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
        }
        h1, h2, h3 {
            color: #ffffff;
        }
        .section {
            background-color: #2a2a2a;
            border-radius: 8px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        .form-group {
            margin-bottom: 15px;
        }
        label {
            display: block;
            margin-bottom: 5px;
            color: #b0b0b0;
        }
        input[type="text"],
        input[type="number"] {
            width: 100%;
            padding: 8px;
            border: 1px solid #444;
            border-radius: 4px;
            background-color: #333;
            color: #e0e0e0;
        }
        .checkbox-group {
            display: flex;
            align-items: center;
            margin-bottom: 15px;
        }
        .checkbox-group label {
            margin-left: 10px;
        }
        button {
            padding: 10px 15px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-weight: bold;
            transition: background-color 0.3s;
        }
        button.start {
            background-color: #4CAF50;
            color: white;
        }
        button.start:hover {
            background-color: #45a049;
        }
        button.stop {
            background-color: #f44336;
            color: white;
        }
        button.stop:hover {
            background-color: #da190b;
        }
        #status {
            margin-top: 20px;
            padding: 10px;
            border-radius: 4px;
            font-weight: bold;
        }
        .debug-section {
            display: flex;
            gap: 20px;
        }
        .debug-panel {
            flex: 1;
            background-color: #2a2a2a;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        #debugLog {
            height: 300px;
            overflow-y: auto;
            font-family: monospace;
            background-color: #333;
            padding: 10px;
            border-radius: 4px;
            white-space: pre-wrap;
            word-wrap: break-word;
        }
        .stats {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            margin-bottom: 15px;
        }
        .stat-box {
            background-color: #333;
            padding: 10px;
            border-radius: 4px;
            text-align: center;
        }
        .stat-label {
            font-weight: bold;
            margin-bottom: 5px;
            color: #b0b0b0;
        }
        .position-info {
            background-color: #333;
            padding: 10px;
            border-radius: 4px;
            margin-top: 10px;
        }
        .tabs {
            display: flex;
            margin-bottom: 20px;
        }
        .tab {
            padding: 10px 20px;
            background-color: #333;
            color: #e0e0e0;
            border: none;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        .tab:hover {
            background-color: #444;
        }
        .tab.active {
            background-color: #4CAF50;
            color: white;
        }
        .tab-content {
            display: none;
        }
        .tab-content.active {
            display: block;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>MAVLink to CoT Converter</h1>
        
        <div class="tabs">
            <button class="tab active" onclick="openTab(event, 'config')">Configuration</button>
            <button class="tab" onclick="openTab(event, 'debug')">Debug Information</button>
        </div>

        <div id="config" class="tab-content active">
            <div class="section">
                <h2>MAVLink Configuration</h2>
                <div class="form-group">
                    <label for="mavlink_port">MAVLink Port:</label>
                    <input type="number" id="mavlink_port" value="14550">
                </div>
                <div class="form-group">
                    <label for="aircraft_name">Aircraft Name:</label>
                    <input type="text" id="aircraft_name" value="FRIENDLY_UAV">
                </div>
            </div>
            
            <div class="section">
                <h2>CoT Configuration</h2>
                <div class="form-group">
                    <label for="cot_ip">CoT IP Address:</label>
                    <input type="text" id="cot_ip" value="239.2.3.1">
                </div>
                <div class="form-group">
                    <label for="cot_port">CoT Port:</label>
                    <input type="number" id="cot_port" value="6969">
                </div>
                <div class="checkbox-group">
                    <input type="checkbox" id="use_multicast" checked>
                    <label for="use_multicast">Use Multicast</label>
                </div>
            </div>
            
            <div class="form-group">
                <button class="start" onclick="startConversion()">Start Conversion</button>
                <button class="stop" onclick="stopConversion()">Stop Conversion</button>
            </div>
            <div id="status"></div>
        </div>

        <div id="debug" class="tab-content">
            <div class="section">
                <h2>Debug Information</h2>
                <div class="debug-section">
                    <div class="debug-panel">
                        <h3>Statistics</h3>
                        <div class="stats">
                            <div class="stat-box">
                                <div class="stat-label">MAVLink Messages</div>
                                <div id="mavlinkCount">0</div>
                            </div>
                            <div class="stat-box">
                                <div class="stat-label">CoT Messages</div>
                                <div id="cotCount">0</div>
                            </div>
                        </div>
                        <h3>Latest Position</h3>
                        <div id="positionInfo" class="position-info">
                            No position data available
                        </div>
                    </div>
                    <div class="debug-panel">
                        <h3>Debug Log</h3>
                        <div id="debugLog"></div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        let lastMavlinkCount = 0;
        let lastCotCount = 0;

        function openTab(evt, tabName) {
            var i, tabcontent, tablinks;
            tabcontent = document.getElementsByClassName("tab-content");
            for (i = 0; i < tabcontent.length; i++) {
                tabcontent[i].style.display = "none";
            }
            tablinks = document.getElementsByClassName("tab");
            for (i = 0; i < tablinks.length; i++) {
                tablinks[i].className = tablinks[i].className.replace(" active", "");
            }
            document.getElementById(tabName).style.display = "block";
            evt.currentTarget.className += " active";
        }

        function updateStatus(message) {
            const status = document.getElementById('status');
            status.textContent = message;
            status.style.backgroundColor = message.includes('success') ? '#4CAF50' : '#f44336';
        }

        function updateDebugInfo() {
            fetch('/debug_info')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('mavlinkCount').textContent = data.mavlink_count;
                    document.getElementById('cotCount').textContent = data.cot_count;
                    
                    if (data.latest_position) {
                        const pos = data.latest_position;
                        document.getElementById('positionInfo').innerHTML = `
                            Time: ${pos.timestamp}<br>
                            Latitude: ${pos.lat.toFixed(6)}°<br>
                            Longitude: ${pos.lon.toFixed(6)}°<br>
                            Altitude: ${pos.alt.toFixed(1)}m<br>
                            Heading: ${pos.heading.toFixed(1)}°
                        `;
                    }

                    if (data.mavlink_count === lastMavlinkCount && data.running) {
                        document.getElementById('mavlinkCount').style.color = '#ff6b6b';
                    } else {
                        document.getElementById('mavlinkCount').style.color = '#e0e0e0';
                    }
                    
                    if (data.cot_count === lastCotCount && data.running) {
                        document.getElementById('cotCount').style.color = '#ff6b6b';
                    } else {
                        document.getElementById('cotCount').style.color = '#e0e0e0';
                    }

                    lastMavlinkCount = data.mavlink_count;
                    lastCotCount = data.cot_count;
                });

            fetch('/debug_messages')
                .then(response => response.json())
                .then(messages => {
                    const debugLog = document.getElementById('debugLog');
                    messages.forEach(msg => {
                        const div = document.createElement('div');
                        div.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
                        debugLog.appendChild(div);
                    });
                    
                    while (debugLog.children.length > 100) {
                        debugLog.removeChild(debugLog.firstChild);
                    }
                    
                    debugLog.scrollTop = debugLog.scrollHeight;
                });
        }

        async function startConversion() {
            const port = document.getElementById('mavlink_port').value;
            const name = document.getElementById('aircraft_name').value;
            const cotIP = document.getElementById('cot_ip').value;
            const cotPort = document.getElementById('cot_port').value;
            const useMulticast = document.getElementById('use_multicast').checked;
            
            const response = await fetch('/start', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    mavlink_port: port,
                    aircraft_name: name,
                    cot_ip: cotIP,
                    cot_port: cotPort,
                    use_multicast: useMulticast
                })
            });
            
            const result = await response.text();
            updateStatus(result);
        }

        async function stopConversion() {
            const response = await fetch('/stop', {
                method: 'POST'
            });
            
            const result = await response.text();
            updateStatus(result);
        }

        setInterval(updateDebugInfo, 1000);
    </script>
</body>
</html>
"""

@app.route('/')
def home():
    return render_template_string(HTML_TEMPLATE)

@app.route('/start', methods=['POST'])
def start():
    data = request.get_json()
    mavlink_port = int(data.get('mavlink_port', 14550))
    aircraft_name = data.get('aircraft_name', 'FRIENDLY_UAV')
    cot_ip = data.get('cot_ip', '239.2.3.1')
    cot_port = int(data.get('cot_port', 6969))
    use_multicast = data.get('use_multicast', True)
    
    result = converter.start_conversion(mavlink_port, aircraft_name, cot_ip, cot_port, use_multicast)
    return result

@app.route('/stop', methods=['POST'])
def stop():
    result = converter.stop_conversion()
    return result

@app.route('/debug_info')
def debug_info():
    return jsonify(converter.get_debug_info())

@app.route('/debug_messages')
def debug_messages():
    return jsonify(converter.get_debug_messages())

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080, debug=True)