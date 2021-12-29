import math

# https://github.com/dgnzlz/Capstone_AWS_DeepRacer

STANDARD_TIME = 50  # seconds (time that is easily done by model)
FASTEST_TIME = 20  # seconds (best time of 1st place on the track)

MIN_SPEED = 1.0
MAX_SPEED = 4.9

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0  # None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        # import math

        ################## HELPER FUNCTIONS ###################

        def get_distance(coor1, coor2):
            return math.sqrt(
                (coor1[0] - coor2[0]) * (coor1[0] - coor2[0])
                + (coor1[1] - coor2[1]) * (coor1[1] - coor2[1])
            )

        def get_radians(coor1, coor2):
            return math.atan2((coor2[1] - coor1[1]), (coor2[0] - coor1[0]))

        def get_degrees(coor1, coor2):
            return math.degrees(get_radians(coor1, coor2))

        def get_diff_radians(angle1, angle2):
            diff = (angle1 - angle2) % (2.0 * math.pi)

            if diff >= math.pi:
                diff -= 2.0 * math.pi

            return diff

        def get_diff_degrees(angle1, angle2):
            return math.degrees(get_diff_radians(angle1, angle2))

        def up_sample(waypoints, factor=20):
            p = waypoints
            n = len(p)

            return [
                [
                    i / factor * p[int((j + 1) % n)][0] + (1 - i / factor) * p[j][0],
                    i / factor * p[int((j + 1) % n)][1] + (1 - i / factor) * p[j][1],
                ]
                for j in range(n)
                for i in range(factor)
            ]

        def get_distance_list(car, waypoints):
            dist_list = []
            min_dist = float("inf")
            min_idx = -1

            for i, waypoint in enumerate(waypoints):
                dist = get_distance(car, waypoint)
                if dist < min_dist:
                    min_dist = dist
                    min_idx = i
                dist_list.append(dist)

            return dist_list, min_dist, min_idx, len(waypoints)

        def detect_bot(params):
            car = [params["x"], params["y"]]

            heading = math.radians(params["heading"])
            track_width = params["track_width"]
            is_reversed = params["is_reversed"]

            objects_location = params["objects_location"]
            objects_left_of_center = params["objects_left_of_center"]

            warned = False
            is_inner = False

            bot_idx = -1
            bot_dist = float("inf")

            for i, location in enumerate(objects_location):
                dist = get_distance(car, location)

                angle = get_radians(car, location)

                diff = abs(get_diff_degrees(heading, angle))

                if dist < track_width and diff < 120:
                    warned = True

                    if dist < bot_dist:
                        bot_idx = i
                        bot_dist = dist

            if warned:
                if is_reversed:
                    if objects_left_of_center[bot_idx] == False:
                        is_inner = True
                else:
                    if objects_left_of_center[bot_idx]:
                        is_inner = True

            return warned, is_inner, bot_dist

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(
                    x1=racing_coords[i][0],
                    x2=car_coords[0],
                    y1=racing_coords[i][1],
                    y2=car_coords[1],
                )
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(
                dist_2_points(
                    x1=closest_coords[0],
                    x2=second_closest_coords[0],
                    y1=closest_coords[1],
                    y2=second_closest_coords[1],
                )
            )

            # Distances between car and closest and second closest racing point
            b = abs(
                dist_2_points(
                    x1=car_coords[0],
                    x2=closest_coords[0],
                    y1=car_coords[1],
                    y2=closest_coords[1],
                )
            )
            c = abs(
                dist_2_points(
                    x1=car_coords[0],
                    x2=second_closest_coords[0],
                    y1=car_coords[1],
                    y2=second_closest_coords[1],
                )
            )

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(
                    -(a ** 4)
                    + 2 * (a ** 2) * (b ** 2)
                    + 2 * (a ** 2) * (c ** 2)
                    - (b ** 4)
                    + 2 * (b ** 2) * (c ** 2)
                    - (c ** 4)
                ) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(
            closest_coords, second_closest_coords, car_coords, heading
        ):

            # Virtually set the car more into the heading direction
            heading_vector = [
                math.cos(math.radians(heading)),
                math.sin(math.radians(heading)),
            ]
            new_car_coords = [
                car_coords[0] + heading_vector[0],
                car_coords[1] + heading_vector[1],
            ]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(
                x1=new_car_coords[0],
                x2=closest_coords[0],
                y1=new_car_coords[1],
                y2=closest_coords[1],
            )
            distance_second_closest_coords_new = dist_2_points(
                x1=new_car_coords[0],
                x2=second_closest_coords[0],
                y1=new_car_coords[1],
                y2=second_closest_coords[1],
            )

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(
            closest_coords, second_closest_coords, car_coords, heading
        ):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(
                closest_coords, second_closest_coords, car_coords, heading
            )

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0]
            )

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(
                first_index, closest_index, len(times_list)
            )

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (
                    current_actual_time / current_expected_time
                ) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [
            [5.46833, -0.29363, 2.7001, 0.0845],
            [5.48009, -0.06753, 2.61826, 0.08647],
            [5.4823, 0.15602, 2.55132, 0.08763],
            [5.47515, 0.37573, 2.5058, 0.08772],
            [5.45879, 0.59058, 2.48511, 0.08671],
            [5.43331, 0.79994, 2.48511, 0.08487],
            [5.39875, 1.00352, 2.48511, 0.08309],
            [5.35509, 1.20137, 2.48511, 0.08153],
            [5.30226, 1.39378, 2.48511, 0.08029],
            [5.24016, 1.58122, 2.48511, 0.07946],
            [5.16869, 1.7642, 2.48962, 0.0789],
            [5.08781, 1.94319, 2.51698, 0.07803],
            [4.99755, 2.11849, 2.56234, 0.07695],
            [4.89806, 2.29024, 2.61874, 0.07579],
            [4.78967, 2.45832, 2.67764, 0.07469],
            [4.67286, 2.62236, 2.68118, 0.07511],
            [4.54826, 2.78182, 2.61946, 0.07726],
            [4.41661, 2.93598, 2.56113, 0.07915],
            [4.27872, 3.08403, 2.51872, 0.08033],
            [4.13537, 3.22519, 2.50435, 0.08034],
            [3.98722, 3.35876, 2.50435, 0.07965],
            [3.83472, 3.48419, 2.50435, 0.07885],
            [3.67803, 3.60115, 2.50435, 0.07807],
            [3.51699, 3.70954, 2.50435, 0.07751],
            [3.35101, 3.80952, 2.50435, 0.07737],
            [3.1791, 3.90148, 2.52935, 0.07708],
            [2.99988, 3.98601, 2.60439, 0.07609],
            [2.81158, 4.06386, 2.73959, 0.07438],
            [2.6121, 4.13589, 2.9443, 0.07203],
            [2.39909, 4.203, 3.22451, 0.06926],
            [2.16993, 4.26612, 3.54931, 0.06697],
            [1.92129, 4.32607, 3.89484, 0.06567],
            [1.64834, 4.38357, 4.0, 0.06973],
            [1.34474, 4.43938, 4.0, 0.07717],
            [1.0011, 4.49435, 4.0, 0.087],
            [0.6036, 4.54952, 4.0, 0.10033],
            [0.13121, 4.60635, 3.76428, 0.1264],
            [-0.4457, 4.66682, 2.79418, 0.2076],
            [-1.16791, 4.73357, 2.2992, 0.31545],
            [-2.02877, 4.8068, 1.99612, 0.43283],
            [-2.90932, 4.88084, 1.77379, 0.49817],
            [-3.56723, 4.92308, 1.60741, 0.41014],
            [-3.95424, 4.92441, 1.46444, 0.26427],
            [-4.22763, 4.90629, 1.337, 0.20493],
            [-4.43843, 4.87618, 1.25133, 0.17017],
            [-4.60914, 4.83767, 1.17921, 0.1484],
            [-4.75111, 4.79274, 1.1072, 0.13449],
            [-4.87108, 4.74268, 1.04616, 0.12426],
            [-4.97309, 4.68835, 1.00981, 0.11445],
            [-5.05966, 4.63033, 1.0, 0.10422],
            [-5.13297, 4.56917, 1.0, 0.09547],
            [-5.1945, 4.50529, 1.0, 0.0887],
            [-5.24506, 4.43893, 1.0, 0.08343],
            [-5.28518, 4.37027, 1.0, 0.07952],
            [-5.31535, 4.2994, 1.0, 0.07702],
            [-5.3361, 4.22634, 1.01878, 0.07455],
            [-5.34796, 4.15093, 1.06875, 0.07143],
            [-5.35147, 4.07279, 1.1555, 0.06769],
            [-5.3471, 3.99127, 1.29507, 0.06304],
            [-5.33536, 3.90552, 1.45328, 0.05955],
            [-5.31628, 3.81422, 1.66271, 0.0561],
            [-5.2896, 3.71555, 1.95889, 0.05218],
            [-5.25496, 3.60753, 2.4232, 0.04681],
            [-5.21249, 3.48891, 3.35015, 0.03761],
            [-5.16411, 3.36153, 4.0, 0.03406],
            [-5.03573, 3.0297, 4.0, 0.08895],
            [-4.90646, 2.69269, 4.0, 0.09024],
            [-4.77614, 2.34966, 4.0, 0.09174],
            [-4.64458, 1.99939, 4.0, 0.09354],
            [-4.51151, 1.64045, 4.0, 0.0957],
            [-4.37662, 1.27096, 4.0, 0.09833],
            [-4.23959, 0.88919, 4.0, 0.1014],
            [-4.1002, 0.49377, 4.0, 0.10482],
            [-3.95806, 0.0825, 4.0, 0.10879],
            [-3.81262, -0.34772, 3.40713, 0.13329],
            [-3.66316, -0.80101, 2.93396, 0.16268],
            [-3.50819, -1.28587, 2.59251, 0.19634],
            [-3.34522, -1.81625, 2.3401, 0.23711],
            [-3.15228, -2.3875, 2.11208, 0.28548],
            [-2.98594, -2.80921, 1.97284, 0.22979],
            [-2.83477, -3.13599, 1.83417, 0.1963],
            [-2.69207, -3.39954, 1.74574, 0.17168],
            [-2.55386, -3.61778, 1.72185, 0.15002],
            [-2.41753, -3.80178, 1.72185, 0.133],
            [-2.28111, -3.95794, 1.72185, 0.12042],
            [-2.14297, -4.09145, 1.72185, 0.11158],
            [-2.00157, -4.20512, 1.72185, 0.10537],
            [-1.85533, -4.30113, 1.72185, 0.1016],
            [-1.7027, -4.3815, 1.75295, 0.0984],
            [-1.54212, -4.44788, 1.83853, 0.09451],
            [-1.37203, -4.50168, 1.97744, 0.09021],
            [-1.19086, -4.54406, 2.14891, 0.08659],
            [-0.99666, -4.57572, 2.3589, 0.08341],
            [-0.78705, -4.59706, 2.55088, 0.0826],
            [-0.55785, -4.60757, 2.779, 0.08256],
            [-0.30245, -4.60611, 3.02969, 0.0843],
            [-0.00818, -4.58991, 3.31677, 0.08886],
            [0.35587, -4.55178, 3.37061, 0.1086],
            [0.94126, -4.45499, 3.22012, 0.18426],
            [1.71643, -4.264, 3.04595, 0.2621],
            [2.25664, -4.08518, 2.9126, 0.19537],
            [2.67972, -3.91562, 2.76933, 0.16458],
            [3.03365, -3.75042, 2.6407, 0.14791],
            [3.33994, -3.58702, 2.54265, 0.13653],
            [3.61053, -3.42385, 2.43352, 0.12984],
            [3.85214, -3.2598, 2.34757, 0.1244],
            [4.06968, -3.09398, 2.31186, 0.11832],
            [4.26621, -2.92563, 2.31186, 0.11193],
            [4.44393, -2.75402, 2.31186, 0.10686],
            [4.60459, -2.57848, 2.31186, 0.10293],
            [4.74916, -2.39829, 2.31186, 0.09993],
            [4.8783, -2.2127, 2.31186, 0.0978],
            [4.99282, -2.02114, 2.32065, 0.09617],
            [5.09356, -1.82328, 2.36655, 0.09382],
            [5.18129, -1.61903, 2.44208, 0.09103],
            [5.25674, -1.40859, 2.53888, 0.08805],
            [5.32052, -1.19252, 2.64704, 0.08511],
            [5.37315, -0.9717, 2.75515, 0.08239],
            [5.41507, -0.7473, 2.85099, 0.08007],
            [5.44664, -0.52076, 2.78738, 0.08206],
        ]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        # all_wheels_on_track = params["all_wheels_on_track"]
        x = params["x"]
        y = params["y"]
        # distance_from_center = params["distance_from_center"]
        # is_left_of_center = params["is_left_of_center"]
        heading = params["heading"]
        progress = params["progress"]
        steps = params["steps"]
        speed = params["speed"]
        steering_angle = params["steering_angle"]
        track_width = params["track_width"]
        # waypoints = params["waypoints"]
        # closest_waypoints = params["closest_waypoints"]
        is_offtrack = params["is_offtrack"]

        # closest_objects = params["closest_objects"]

        ############### OPTIMAL X,Y,SPEED,TIME ################

        track = racing_track

        # if closest_objects:
        #     warned, is_inner, _ = detect_bot(params)

        #     if warned:
        #         if is_inner:
        #             track = outer_track
        #         else:
        #             track = inner_track

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            track, [x, y]
        )

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = track[closest_index]
        optimals_second = track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1
        MIN_REWARD = 1e-2

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 2
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(MIN_REWARD, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        need_speed = optimals[2]
        if need_speed < MIN_SPEED:
            need_speed = MIN_SPEED
        elif need_speed > MAX_SPEED:
            need_speed = MAX_SPEED

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 3
        speed_diff = abs(need_speed - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1.5
        # STANDARD_TIME = 50  # seconds (time that is easily done by model)
        # FASTEST_TIME = 20  # seconds (best time of 1st place on the track)
        times_list = [row[3] for row in track]
        projected_time = projected_time(
            self.first_racingpoint_index, closest_index, steps, times_list
        )
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(
                MIN_REWARD,
                (
                    -REWARD_PER_STEP_FOR_FASTEST_TIME
                    * (FASTEST_TIME)
                    / (STANDARD_TIME - FASTEST_TIME)
                )
                * (steps_prediction - (STANDARD_TIME * 15 + 1)),
            )
            steps_reward = min(
                REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction
            )
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading
        )
        if direction_diff > 30 or abs(steering_angle) > 20:
            reward = MIN_REWARD
        else:
            reward += 1.1 - (direction_diff / 30)

        # Zero reward of obviously too slow
        speed_diff_zero = need_speed - speed
        if speed_diff_zero > 0.5:
            reward = MIN_REWARD

        ## Incentive for finishing the lap in less steps ##
        # should be adapted to track length and other rewards
        REWARD_FOR_FASTEST_TIME = 300
        # STANDARD_TIME = 50  # seconds (time that is easily done by model)
        # FASTEST_TIME = 20  # seconds (best time of 1st place on the track)
        if progress > 99.5:
            finish_reward = max(
                MIN_REWARD,
                (-REWARD_FOR_FASTEST_TIME / (15 * (STANDARD_TIME - FASTEST_TIME)))
                * (steps - STANDARD_TIME * 15),
            )
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if is_offtrack == True:
            reward = MIN_REWARD

        ####################### VERBOSE #######################
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
