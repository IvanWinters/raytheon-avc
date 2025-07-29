import numpy as np

def generate_rose_waypoints(n_points, k=5, a=5):
    """
    :param n_points: Number of waypoints
    :param k: Petal parameter of the rose curve
    :param a: Scaling factor
    :return: List of waypoint strings
    """
    waypoints = []
    theta = np.linspace(0, 2 * np.pi, n_points)
    r = a * np.cos(k * theta)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = 4  # Fixed altitude
    
    for i in range(n_points):
        waypoint = (f"nextWayPoint.x = {x[i]:.2f};\n"
                    f"\tnextWayPoint.y = {y[i]:.2f};\n"
                    f"\tnextWayPoint.z = {z};\n"
                    f"\twaypointList.push_back(nextWayPoint);\n")
        waypoints.append(waypoint)
    
    return waypoints

if __name__ == "__main__":
    n_points = int(input("Enter the number of waypoints: "))
    waypoints = generate_rose_curve_waypoints(n_points)
    for wp in waypoints:
        print(wp)
