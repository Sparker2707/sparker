import asyncio
import csv
import threading
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene
from PyQt5.QtGui import QPen
from PyQt5.QtCore import Qt
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

class DronePathWindow(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.setWindowTitle("Drone Pathway")
        self.setGeometry(100, 100, 800, 600)
        self.pen = QPen(Qt.blue)
        self.pen.setWidth(2)
        self.prev_x, self.prev_y = None, None

    def update_trajectory(self, px, py):
        """Update the trajectory in real-time."""
        x = px * 10  # Scale for visualization
        y = py * 10
        if self.prev_x is not None and self.prev_y is not None:
            self.scene.addLine(self.prev_x, self.prev_y, x, y, self.pen)
        self.prev_x, self.prev_y = x, y

def read_csv(file_path):
    trajectory_data = []
    with open(file_path, mode="r") as file:
        reader = csv.DictReader(file)
        for row in reader:
            trajectory_data.append(
                {
                    "t": float(row["t"]),
                    "px": float(row["px"]),
                    "py": float(row["py"]),
                    "pz": float(row["pz"]),
                    "vx": float(row["vx"]),
                    "vy": float(row["vy"]),
                    "vz": float(row["vz"]),
                    "yaw": float(row["yaw"]),
                }
            )
    return trajectory_data

async def run_offboard(file_path, update_gui_callback):
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    print("Arming the drone...")
    await drone.action.arm()
    takeoff_altitude = 10.0  # Desired altitude in meters
    await drone.action.set_takeoff_altitude(takeoff_altitude)
    print(f"Takeoff altitude set to {takeoff_altitude} meters.")

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print("Setting up offboard control...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    try:
        await drone.offboard.start()
        print("Offboard started!")
    except OffboardError as error:
        print(f"Failed to start offboard: {error}")
        await drone.action.disarm()
        return

    trajectory = read_csv(file_path)
    print(f"Trajectory loaded: {len(trajectory)} points")

    previous_point = None
    for point in trajectory:
        pz = max(point["pz"], takeoff_altitude)  # Prevent reducing altitude below takeoff altitude
        position = PositionNedYaw(point["px"], point["py"], -pz, point["yaw"])

        if previous_point and (
            previous_point["px"] == point["px"]
            and previous_point["py"] == point["py"]
            and previous_point["pz"] == pz
            and previous_point["yaw"] == point["yaw"]
        ):
            continue

        await drone.offboard.set_position_ned(position)
        print(f"Moving to: PX={point['px']}, PY={point['py']}, PZ={pz}, Yaw={point['yaw']}")

        # Update the PyQt5 GUI
        update_gui_callback(point["px"], point["py"])

        await asyncio.sleep(0.1)
        previous_point = {"px": point["px"], "py": point["py"], "pz": pz, "yaw": point["yaw"]}

    print("Trajectory completed. Returning to launch...")
    await drone.action.return_to_launch()
    await asyncio.sleep(10)

    print("Disarming the drone...")
    await drone.action.disarm()

def main():
    file_path = "/home/seiger/mavsdk_drone_show/shapes/newposi.csv"
    app = QApplication([])
    window = DronePathWindow()

    # Create an asyncio loop in a separate thread
    def run_asyncio():
        asyncio.run(run_offboard(file_path, window.update_trajectory))

    asyncio_thread = threading.Thread(target=run_asyncio, daemon=True)
    asyncio_thread.start()

    window.show()
    app.exec_()

if __name__ == "__main__":
    main()
