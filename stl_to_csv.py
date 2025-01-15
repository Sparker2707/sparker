import csv
import math
import trimesh
import numpy as np

def stl_to_waypoints(stl_file, output_csv, speed=0.1, acceleration=0.5, altitude_threshold=5.0, hover_time=5.0, waypoint_hold_time=2.0):
    try:
        scene = trimesh.load_mesh(stl_file)
        if isinstance(scene, trimesh.Scene):
            mesh = scene.geometry[list(scene.geometry.keys())[0]]
        else:
            mesh = scene

        mesh.process()
        mesh_center = mesh.centroid
        hull = mesh.convex_hull
        perimeter_vertices = hull.vertices

        with open(output_csv, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['idx', 't', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'yaw', 'mode'])

            idx = 0
            t = 0.0
            last_vertex = None
            last_velocity = (0, 0, 0)
            last_acceleration = (0,0,0)

            if perimeter_vertices.size > 0:
                first_vertex = perimeter_vertices[0]
                x_offset, y_offset, z_offset = first_vertex - mesh_center
                px = x_offset
                py = y_offset
                pz = altitude_threshold

                writer.writerow([idx, t, px, py, pz, 0, 0, 0, 0, 0, 0, 0, 10])
                idx += 1
                t += hover_time

                last_vertex = (px, py, pz)

                for vertex in perimeter_vertices:
                    x_offset, y_offset, z_offset = vertex - mesh_center
                    px = x_offset
                    py = y_offset
                    pz = altitude_threshold

                    if last_vertex is not None:
                        delta_x = px - last_vertex[0]
                        delta_y = py - last_vertex[1]
                        delta_z = pz - last_vertex[2]
                        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
                        
                        # Calculate velocity components
                        if distance > 0:
                            vx = (delta_x / distance) * speed
                            vy = (delta_y / distance) * speed
                            vz = (delta_z / distance) * speed
                        else:
                            vx, vy, vz = 0, 0, 0
                        
                        # Calculate acceleration components (simplified)
                        delta_vx = vx - last_velocity[0]
                        delta_vy = vy - last_velocity[1]
                        delta_vz = vz - last_velocity[2]

                        if (t - (t-waypoint_hold_time - (distance/speed) )) >0:
                            ax = delta_vx / (t - (t-waypoint_hold_time - (distance/speed) ))
                            ay = delta_vy / (t - (t-waypoint_hold_time - (distance/speed) ))
                            az = delta_vz / (t - (t-waypoint_hold_time - (distance/speed) ))
                        else:
                            ax, ay, az = 0, 0, 0

                        t += distance / speed
                        
                    else:
                        vx, vy, vz = 0, 0, 0
                        ax, ay, az = 0, 0, 0

                    yaw = 0
                    mode = 10
                    writer.writerow([idx, t, px, py, pz, vx, vy, vz, ax, ay, az, yaw, mode])
                    idx += 1
                    t += waypoint_hold_time
                    last_vertex = (px, py, pz)
                    last_velocity = (vx, vy, vz)
                    last_acceleration = (ax, ay, az)

            else:
                print("No vertices found in the mesh")

        print(f"Waypoints saved to {output_csv}")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    stl_file_path = "/mnt/c/Users/sanja/Downloads/box.stl"   #change path and file name
    output_csv = "/mnt/c/Users/sanja/Downloads/waypoints.csv"  #give destination path
    altitude_threshold = 5.0
    stl_to_waypoints(stl_file_path, output_csv, altitude_threshold=altitude_threshold)
