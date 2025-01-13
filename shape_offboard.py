import asyncio
import csv
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

# Read the CSV File
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

# Offboard Control Function
async def run_offboard(file_path):
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

    # Take off to the specified altitude
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

    # Load the trajectory from the CSV file
    trajectory = read_csv(file_path)
    print(f"Trajectory loaded: {len(trajectory)} points")

    # Follow the trajectory
    previous_point = None  # Track the last point sent to the drone
    for point in trajectory:
        # Ensure the altitude does not go below the takeoff altitude
        pz = max(point["pz"], takeoff_altitude)  # Prevent reducing altitude below takeoff altitude
        position = PositionNedYaw(point["px"], point["py"], -pz, point["yaw"])

        # Check if the current point is the same as the previous one
        if previous_point and (
            previous_point["px"] == point["px"]
            and previous_point["py"] == point["py"]
            and previous_point["pz"] == pz
            and previous_point["yaw"] == point["yaw"]
        ):
            # Skip this point as it's identical to the previous one
            continue

        # Set position or velocity commands
        await drone.offboard.set_position_ned(position)

        print(f"Moving to: PX={point['px']}, PY={point['py']}, PZ={pz}, Yaw={point['yaw']}")
        await asyncio.sleep(0.1)  # Adjust the sleep duration based on the CSV timestep

        # Update the previous point
        previous_point = {"px": point["px"], "py": point["py"], "pz": pz, "yaw": point["yaw"]}

    print("Trajectory completed. Returning to launch...")
    await drone.action.return_to_launch()
    await asyncio.sleep(10)

    print("Disarming the drone...")
    await drone.action.disarm()

# Run the script
if __name__ == "__main__":
    file_path = "/home/seiger/mavsdk_drone_show/shapes/newposi.csv"
    asyncio.run(run_offboard(file_path))

