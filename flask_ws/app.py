from flask import Flask, render_template, redirect, url_for, jsonify, request, Response
from flask_socketio import SocketIO
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import NavSatFix
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import subprocess
import cv2
import numpy as np
import threading
import time
import atexit
import signal
import os
import yaml

app = Flask(__name__)

# Use a flag to simulate local testing
IS_LOCAL = False  # Set to False when running on the Jetson
gps_data = {"latitude": 35.307594, "longitude": -80.731187}
geofence_data = []

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher = self.create_publisher(String, '/robot_status', 10)

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Status Updated: {message}')

# Camera section
# Global video capture objects for two cameras
# Front camera stream from ZED 2i

front_camera = cv2.VideoCapture('/dev/video0')
back_camera = cv2.VideoCapture('/dev/video2')
print("Front camera opened:", front_camera.isOpened())
print("Back camera opened:", back_camera.isOpened())

def generate_feed(camera):
    while True:
        success, frame = camera.read()
        if not success:
            break
                # Crop only the left half (adjust if needed)
        width = frame.shape[1] // 2
        height = frame.shape[0]
        left_frame = frame[0:height, 0:width]  # Crop left half

        # Optional: resize for web display
        resized = cv2.resize(left_frame, (400, 300))

        _, buffer = cv2.imencode('.jpg', resized)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# Cleanup on exit
@atexit.register
def cleanup():
    front_camera.release()
    back_camera.release()

@app.route('/video_feed/front')
def video_feed_front():
    if IS_LOCAL:
        return redirect(url_for('static', filename='placeholder.jpg'))
    else:
        return Response(generate_feed(front_camera),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed/back')
def video_feed_back():
    if IS_LOCAL:
        return redirect(url_for('static', filename='placeholder.jpg'))
    else:
        return Response(generate_feed(back_camera),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    
# GPS Section
class GPSListener(Node):
    def __init__(self):
        super().__init__('gps_listener')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps_data',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global gps_data
        gps_data["latitude"] = msg.latitude
        gps_data["longitude"] = msg.longitude

# Geofence drawing
@app.route('/save_geofence', methods=['POST'])
def save_geofence():
    global geofence_data
    geofence_data = request.json.get('points', [])
    print("Received geofence:", geofence_data)

    # Save to YAML file
    try:
        with open('/home/epriauto/flask_ws/geofence.yaml', 'w') as file:
            yaml.dump({'geofence': geofence_data}, file)
        print("Geofence saved to geofence.yaml")
    except Exception as e:
        print("Failed to save geofence:", e)

    return jsonify({"status": "success", "points": geofence_data})

# Load geofence from yaml
@app.route('/load_geofence', methods=['GET'])
def load_geofence():
    try:
        print("Trying to load geofence from YAML...")
        with open('/home/epriauto/flask_ws/geofence.yaml', 'r') as file:
            data = yaml.safe_load(file)
        print("Geofence loaded:", data)
        return jsonify(data)
    except Exception as e:
        print("Failed to load geofence:", e)
        return jsonify({"error": str(e)}), 500


# Manual load geofence
@app.route('/get_geofence', methods=['GET'])
def get_geofence():
    global geofence_data
    return jsonify(geofence_data)


# Clear geofence
@app.route('/clear_geofence', methods=['POST'])
def clear_geofence():
    global geofence_data
    geofence_data = []
    # Also delete the saved file if you're using persistent storage
    import os
    try:
        os.remove('geofence.json')
    except FileNotFoundError:
        pass
    return jsonify({"message": "Geofence cleared"})

# Battery Section
battery_percentage_value = 100
class BatteryListener(Node):
    def __init__(self):
        super().__init__('battery_listener')
        self.subscription = self.create_subscription(
            Int32,
            '/battery_percentage',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global battery_percentage_value
        battery_percentage_value = msg.data


# Message output section
robot_status_message = "Current Mode: Idle"
class StatusListener(Node):
    def __init__(self):
        super().__init__('status_listener')
        self.subscription = self.create_subscription(
            String,
            '/robot_status',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global robot_status_message
        robot_status_message = msg.data


@app.route("/")
def home():
    return render_template("index.html")

@app.route('/run_lidar')
def run_lidar():
    command = "cd && /usr/local/bin/init_lidar.sh"
    subprocess.Popen(["bash", "-c", command])  # Don't block Flask
    return redirect(url_for('home'))

@app.route('/gps_data')
def gps_data_endpoint():
    return jsonify(gps_data)

@app.route('/run_robot')
def run_robot():
    command = "cd && /usr/local/bin/run_all.sh"
    subprocess.Popen(["bash", "-c", command])  # Don't block Flask

    return redirect(url_for('home'))

@app.route('/manual_mode')
def manual_mode():
    command = "cd && /usr/local/bin/manual_motor.sh"
    subprocess.Popen(["bash", "-c", command])  # Don't block Flask

    return redirect(url_for('home'))

@app.route('/emergency_stop')
def emergency_stop():

    os.system("pkill -f terminator")

    return jsonify({"status": "Emergency stop: All terminals closed."})


#Battery reading section
@app.route('/battery_percentage')
def battery_percentage():
    return {"percentage": battery_percentage_value}

# Message output section
@app.route('/robot_status')
def robot_status():
    return {"status": robot_status_message}

# Start ROS2 node
def start_ros2_node():
     rclpy.init()
     battery_node = BatteryListener()
     status_publish = StatusPublisher()
     status_node = StatusListener()
     gps_node = GPSListener()

     executor = MultiThreadedExecutor()
     executor.add_node(battery_node)
     executor.add_node(status_publish)
     executor.add_node(status_node)
     executor.add_node(gps_node)

     def spin():
        executor.spin()
        battery_node.destroy_node()
        status_node.destroy_node()
        status_publish.destroy_node()
        gps_node.destroy_node()
        rclpy.shutdown()

     thread = threading.Thread(target=spin, daemon=True)
     thread.start()


if __name__ == '__main__':
    start_ros2_node()
    app.run(host='192.168.1.101')