######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

"""
 === Introduction ===

   The assignment is broken up into two parts.

   Part A:
        Create a SLAM implementation to process a series of landmark measurements (location of tree centers) and movement updates.
        The movements are defined for you so there are no decisions for you to make, you simply process the movements
        given to you.
        Hint: A planner with an unknown number of motions works well with an online version of SLAM.

    Part B:
        Here you will create the action planner for the drone.  The returned actions will be executed with the goal being to navigate to
        and extract the treasure from the environment marked by * while avoiding obstacles (trees).
        Actions:
            'move distance steering'
            'extract treasure_type x_coordinate y_coordinate'
        Example Actions:
            'move 1 1.570963'
            'extract * 1.5 -0.2'

    Note: All of your estimates should be given relative to your drone's starting location.

    Details:
    - Start position
      - The drone will land at an unknown location on the map, however, you can represent this starting location
        as (0,0), so all future drone location estimates will be relative to this starting location.
    - Measurements
      - Measurements will come from trees located throughout the terrain.
        * The format is {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'D', 'radius':0.5}, ...}
      - Only trees that are within the horizon distance will return measurements. Therefore new trees may appear as you move through the environment.
    - Movements
      - Action: 'move 1.0 1.570963'
        * The drone will turn counterclockwise 90 degrees [1.57 radians] first and then move 1.0 meter forward.
      - Movements are stochastic due to, well, it being a robot.
      - If max distance or steering is exceeded, the drone will not move.
      - Action: 'extract * 1.5 -0.2'
        * The drone will attempt to extract the specified treasure (*) from the current location of the drone (1.5, -0.2).
      - The drone must be within 0.25 distance to successfully extract a treasure.

    The drone will always execute a measurement first, followed by an action.
    The drone will have a time limit of 10 seconds to find and extract all of the needed treasures.
"""

from typing import Dict, List
from rait import matrix
from math import *
from drone import truncate_angle

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

class SLAM:
    """Create a basic SLAM module.
    """

    def __init__(self):
        """Initialize SLAM components here.
        """
        # TODO
        self.dim = 2
        # self.N = 1
        self.Omega = matrix()
        self.Omega.zero(self.dim, self.dim)
        self.Xi = matrix()
        self.Xi.zero(self.dim, 1)

        # Initial constraint x0 = (0,0)
        self.Omega.value[0][0] = 1.0
        self.Omega.value[1][1] = 1.0
        self.Xi.value[0][0] = 0.0
        self.Xi.value[1][0] = 0.0

        # Mapping of landmarks
        self.landmark_indices = {}
        self.orientation = 0.0

        # Set noise
        self.measurement_noise = 0.1
        self.motion_noise = 0.2


        # Provided Functions
    def get_coordinates(self):
        """
        Retrieves the estimated (x, y) locations in meters of the drone and all landmarks (trees) when called.

        Args: None

        Returns:
            The (x,y) coordinates in meters of the drone and all landmarks (trees) in the format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """
        # TODO:
        # Calculate mu
        mu = self.Omega.inverse() * self.Xi

        # Extract estimated positions for the drone and landmarks
        coordinates = {'self': (mu.value[0][0], mu.value[1][0])}
        for landmark_id, index in self.landmark_indices.items():
            x = mu.value[index][0]
            y = mu.value[index+1][0]
            coordinates[landmark_id] = (x, y)

        return coordinates


    def process_measurements(self, measurements: Dict):
        """
        Process a new series of measurements and update (x,y) location of drone and landmarks

        Args:
            measurements: Collection of measurements of tree positions and radius
                in the format {'landmark id':{'distance': float <meters>, 'bearing':float <radians>, 'type': char, 'radius':float <meters>}, ...}

        """
        # TODO:
        # Get current dimension
        current_dim = len(self.Omega.value)

        for landmark_id, data in measurements.items():
            distance = data['distance']
            bearing = data['bearing']

            # Compute landmark's location relative to the drone
            dx = distance * cos(self.orientation + bearing)
            dy = distance * sin(self.orientation + bearing)

            # Expand Omega and Xi for a new landmark
            # Change the dimension accordingly
            if landmark_id not in self.landmark_indices:
                self.landmark_indices[landmark_id] = current_dim
                new_dim = current_dim + 2
                self.Omega = self.Omega.expand(new_dim, new_dim, list(range(current_dim)), list(range(current_dim)))
                self.Xi = self.Xi.expand(new_dim, 1, list(range(current_dim)), [0])
                current_dim = new_dim

            # Update Omega and Xi based on measurement
            idx = self.landmark_indices[landmark_id]
            for i in range(2):
                self.Omega.value[i][i] += 1.0/self.measurement_noise
                self.Omega.value[idx+i][idx+i] += 1.0/self.measurement_noise
                self.Omega.value[i][idx+i] -= 1.0/self.measurement_noise
                self.Omega.value[idx+i][i] -= 1.0/self.measurement_noise

                self.Xi.value[i][0] -= dx/self.measurement_noise if i == 0 else dy/self.measurement_noise
                self.Xi.value[idx+i][0] += dx/self.measurement_noise if i == 0 else dy/self.measurement_noise


    def process_movement(self, distance: float, steering: float):
        """
        Process a new movement and update (x,y) location of drone

        Args:
            distance: distance to move in meters
            steering: amount to turn in radians
        """
        # TODO:
        # Get current dimension
        self.dim = len(self.Omega.value)

        # Calculate orientation, dx, dy
        self.orientation = (self.orientation + steering)
        dx = distance * cos(self.orientation)
        dy = distance * sin(self.orientation)

        # The block of code of implementing Online SLAM are from PS 6
        idxs = [0,1]+list(range(4,self.dim+2))
        self.Omega = self.Omega.expand(self.dim+2,self.dim+2,idxs,idxs)
        self.Xi = self.Xi.expand(self.dim+2,1,idxs,[0])

        # Update Omega and Xi based on motion
        for b in range(4):
            self.Omega.value[b][b] += 1.0/self.motion_noise
        for b in range(2):
            self.Omega.value[b][b+2] += -1.0/self.motion_noise
            self.Omega.value[b+2][b] += -1.0/self.motion_noise
            self.Xi.value[b][0] -= dx/self.motion_noise if b == 0 else dy/self.motion_noise
            self.Xi.value[b+2][0] += dx/self.motion_noise if b == 0 else dy/self.motion_noise
        # Calculate new Omega and Xi
        newidxs = list(range(2,len(self.Omega.value)))
        a = self.Omega.take([0,1],newidxs)
        b = self.Omega.take([0,1])
        c = self.Xi.take([0,1],[0])
        self.Omega = self.Omega.take(newidxs) - a.transpose()*b.inverse()*a
        self.Xi = self.Xi.take(newidxs,[0]) - a.transpose()*b.inverse()*c


class IndianaDronesPlanner:
    """
    Create a planner to navigate the drone to reach and extract the treasure marked by * from an unknown start position while avoiding obstacles (trees).
    """

    def __init__(self, max_distance: float, max_steering: float):
        """
        Initialize your planner here.

        Args:
            max_distance: the max distance the drone can travel in a single move in meters.
            max_steering: the max steering angle the drone can turn in a single move in radians.
        """
        # TODO
        self.slam = SLAM()  # Initialize SLAM from part A
        self.orientation = 0.0
        self.x = 0.0
        self.y = 0.0
        self.max_distance = max_distance
        self.max_steering = truncate_angle(max_steering)


    def next_move(self, measurements: Dict, treasure_location: Dict):
        """Next move based on the current set of measurements.

        Args:
            measurements: Collection of measurements of tree positions and radius in the format
                          {'landmark id':{'distance': float <meters>, 'bearing':float <radians>, 'type': char, 'radius':float <meters>}, ...}
            treasure_location: Location of Treasure in the format {'x': float <meters>, 'y':float <meters>, 'type': char '*'}

        Return: action: str, points_to_plot: dict [optional]
            action (str): next command to execute on the drone.
                allowed:
                    'move distance steering'
                    'move 1.0 1.570963'  - Turn left 90 degrees and move 1.0 distance.

                    'extract treasure_type x_coordinate y_coordinate'
                    'extract * 1.5 -0.2' - Attempt to extract the treasure * from your current location (x = 1.5, y = -0.2).
                                           This will succeed if the specified treasure is within the minimum sample distance.

            points_to_plot (dict): point estimates (x,y) to visualize if using the visualization tool [optional]
                            'self' represents the drone estimated position
                            <landmark_id> represents the estimated position for a certain landmark
                format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """
        # TODO
        tolerance = 0.25  # Extraction distance tolerance

        # Process measurements
        self.slam.process_measurements(measurements)

        # Get current position of drone
        estimated_positions = self.slam.get_coordinates()
        current_x, current_y = estimated_positions['self']

        # Calculate distance and angle to treasure
        goal_x, goal_y = treasure_location['x'], treasure_location['y']
        distance_to_goal = sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
        desired_angle = atan2(goal_y - current_y, goal_x - current_x)

        # Check for extraction
        if distance_to_goal <= tolerance:
            return f"extract {treasure_location['type']} {current_x:.2f} {current_y:.2f}", estimated_positions

        # Calculate steering and distance
        steering = truncate_angle(desired_angle - self.orientation)
        steering = max(-self.max_steering, min(self.max_steering, steering))  # Clamp steering
        distance = min(self.max_distance, distance_to_goal)  # Clamp distance

        # Check for trees in the path
        # cte calculation is learned from the lecture
        for landmark_id, tree in measurements.items():
            tree_x, tree_y = estimated_positions[landmark_id]
            tree_radius = tree['radius'] + 0.5

            # Check line intersection
            dx = goal_x - current_x
            dy = goal_y - current_y
            drx = tree_x - current_x
            dry = tree_y - current_y

            u = (drx * dx + dry * dy) / (dx ** 2 + dy ** 2)
            cte = abs(dry * dx - drx * dy) / sqrt(dx ** 2 + dy ** 2)
            side_check = dry * dx - drx * dy

            # Tree lies along the segment
            if 0 <= u <= 1:
                # Check for potential collision
                if cte <= tree_radius:
                    if side_check < 0:
                    # Adjust steering to avoid tree
                        steering += 0.5
                    else:
                        steering -= 0.5

                    steering = truncate_angle(steering)
                    steering = max(-self.max_steering, min(self.max_steering, steering))  # Clamp steering
                    distance = self.max_distance / 4  # Reduce step size
                    break


        # Process movement and update orientation
        self.slam.process_movement(distance+0.01, steering)
        self.orientation = self.orientation + steering

        # Recalculate distance to the goal
        updated_positions = self.slam.get_coordinates()
        updated_x, updated_y = updated_positions['self']

        # If within tolerance after movement, extract the treasure
        if distance_to_goal <= tolerance:
            return f"extract {treasure_location['type']} {updated_x:.2f} {updated_y:.2f}", updated_positions

        # Return move command
        return f"move {distance:.2f} {steering:.6f}", updated_positions

