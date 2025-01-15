#!/usr/bin/env python3

import asyncio
import csv
import logging
from mavsdk import System
from mavsdk.offboard import (PositionNedYaw, OffboardError)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("offboard_example")

async def run(csv_path):
    """Fly the drone through waypoints and then return to position mode."""

    drone = System(mavsdk_server_address='localhost', port=50051)
    await drone.connect(system_address="udp://:14540")

    logger.info("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            logger.info(f"-- Connected to drone!")
            break

    logger.info("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            logger.info("-- Global position estimate OK")
            break

    logger.info("-- Arming")
    try:
        await drone.action.arm()
    except Exception as error:
        logger.error(f"Arming failed: {error}")
        return

    logger.info("-- Taking off")
    try:
        await drone.action.takeoff()
        await asyncio.sleep(5)  # Give it time to take off
    except Exception as error:
        logger.error(f"Takeoff failed: {error}")
        return

    try:
        with open(csv_path, mode='r') as file:
            reader = csv.DictReader(file)
            first_waypoint = next(reader)
            px, py, pz = float(first_waypoint['px']), float(first_waypoint['py']), float(first_waypoint['pz'])
            logger.info(f"-- Setting initial setpoint to px={px}, py={py}, pz={pz}")
            await drone.offboard.set_position_ned(PositionNedYaw(px, py, -pz, 0)) # -pz for NED
    except FileNotFoundError:
        logger.error(f"Error: The CSV file {csv_path} was not found.")
        return
    except Exception as e:
        logger.error(f"An error occurred while reading waypoints: {e}")
        return

    logger.info("-- Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        logger.error(f"Starting offboard mode failed: {error}")
        await drone.action.disarm()
        return

    logger.info("-- Flying through waypoints")
    try:
        with open(csv_path, mode='r') as file:
            reader = csv.DictReader(file)
            next(reader, None)  # Skip header
            for row in reader:
                try:
                    px = float(row['px'])
                    py = float(row['py'])
                    pz = float(row['pz'])
                    logger.info(f"Flying to waypoint: px={px}, py={py}, pz={pz}")
                    await drone.offboard.set_position_ned(PositionNedYaw(px, py, -pz, 0)) # -pz for NED
                    await asyncio.sleep(5)  # Adjust as needed
                except ValueError:
                    logger.error(f"Error parsing waypoint data. Skipping this waypoint.")
    except Exception as e:
        logger.error(f"An error occurred while flying to waypoints: {e}")

    logger.info("-- Stopping offboard and returning to Position mode")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        logger.error(f"Stopping offboard mode failed: {error}")
        await drone.action.disarm()
        return

    # No need to explicitly set position mode; stopping offboard does this automatically.
    # We can still land if needed.
    await drone.action.land()


if __name__ == "__main__":
    csv_file_path = "/mnt/c/Users/sanja/Downloads/waypoints.csv"  #stl csv file
    asyncio.run(run(csv_file_path))
