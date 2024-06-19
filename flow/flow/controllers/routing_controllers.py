"""Contains a list of custom routing controllers."""
import random
import numpy as np

from flow.controllers.base_routing_controller import BaseRouter

class ContinuousRouter(BaseRouter):
    """A router used to continuously re-route of the vehicle in a closed ring.

    This class is useful if vehicles are expected to continuously follow the
    same route, and repeat said route once it reaches its end.

    Usage
    -----
    See base class for usage example.
    """

    def choose_route(self, env):
        """See parent class.

        Adopt one of the current edge's routes if about to leave the network.
        """
        edge = env.k.vehicle.get_edge(self.veh_id)
        current_route = env.k.vehicle.get_route(self.veh_id)

        if len(current_route) == 0:
            # this occurs to inflowing vehicles, whose information is not added
            # to the subscriptions in the first step that they departed
            return None
        elif edge == current_route[-1]:
            # choose one of the available routes based on the fraction of times
            # the given route can be chosen
            num_routes = len(env.available_routes[edge])
            frac = [val[1] for val in env.available_routes[edge]]
            route_id = np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0]

            # pass the chosen route
            return env.available_routes[edge][route_id][0]
        else:
            return None

class MinicityRouter(BaseRouter):
    """A router used to continuously re-route vehicles in minicity network.

    This class allows the vehicle to pick a random route at junctions.

    Usage
    -----
    See base class for usage example.
    """

    def choose_route(self, env):
        """See parent class."""
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0

        if len(veh_next_edge) == no_next:
            next_route = None
        elif veh_route[-1] == veh_edge:
            random_route = random.randint(0, len(veh_next_edge) - 1)
            while veh_next_edge[0][0][0] == not_an_edge:
                veh_next_edge = env.k.network.next_edge(
                    veh_next_edge[random_route][0],
                    veh_next_edge[random_route][1])
            next_route = [veh_edge, veh_next_edge[0][0]]
        else:
            next_route = None

        if veh_edge in ['e_37', 'e_51']:
            next_route = [veh_edge, 'e_29_u', 'e_21']

        return next_route

class RingBusHumanRouter(BaseRouter):
    """RL cars will choose the shortest paths whenever possible
    """

    def choose_route(self, env):
        """See parent class."""
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0
        prob = 1 # probability to select the shortest pass
        if len(veh_next_edge) == no_next:
            next_route = None
        elif veh_edge == 'bottom':
            if np.random.uniform(0,1,1)[0] <= prob:
                next_route = ['bottom','central'] # always go central 
            else:
                next_route = ['bottom','right'] # do not go central

        elif veh_edge == veh_route[-1]:
            
            num_routes = len(env.available_routes[veh_edge])
            frac = [val[1] for val in env.available_routes[veh_edge]]
            route_id = np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0]

            next_route = env.available_routes[veh_edge][route_id][0]
        else:
            next_route = None

        return next_route

class RingBusHumanRouterGreedy(BaseRouter):
    """RL cars will choose the shortest paths whenever possible
    """

    def choose_route(self, env):
        """See parent class."""
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0
        if len(veh_next_edge) == no_next:
            next_route = None
        elif veh_edge == 'right':
            #Greedy selection
            
            # Define central velocity
            central_vehs = env.k.vehicle.get_ids_by_edge('central')
            if len(central_vehs)>0:
                sorted_central_vehs =sorted(central_vehs, key=env._get_abs_position)
                central_velocity = env.k.vehicle.get_speed(sorted_central_vehs[0])
            else:
                central_velocity = 30 #maximum speed
            # Define top velocity
            top_vehs = env.k.vehicle.get_ids_by_edge('top')
            if len(top_vehs)>0:
                sorted_top_vehs =sorted(top_vehs, key=env._get_abs_position)
                top_velocity = env.k.vehicle.get_speed(sorted_top_vehs[0])
            else:
                top_velocity = 30 #maximum speed

            right_vehs = env.k.vehicle.get_ids_by_edge('right')
            if len(right_vehs)>0:
                sorted_right_vehs =sorted(right_vehs, key=env._get_abs_position)
                right_velocity = env.k.vehicle.get_speed(sorted_right_vehs[0])
            else:
                right_velocity = 30 #maximum speed
            # print(f'*******top: {top_velocity}, right: {right_velocity}, central: {central_velocity}')
            outer_velocity = top_velocity + right_velocity
            # (np.percentile(outer_velocity, 80) > np.percentile(central_velocity, 80))
            # if  len(central_vehs) > 3 * len(top_vehs + right_vehs):
            if top_velocity > central_velocity:
                next_route = ['right','top'] # do not go central
            else:
                next_route = ['right','central'] #  go central 
        elif veh_edge == veh_route[-1]:
            
            num_routes = len(env.available_routes[veh_edge])
            frac = [val[1] for val in env.available_routes[veh_edge]]
            route_id = np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0]

            next_route = env.available_routes[veh_edge][route_id][0]
        else:
            next_route = None

        return next_route

from flow.networks.gridmerge import ADDITIONAL_NET_PARAMS

def sigmoid(x):
    return 1 / (1 + (2.75)**(-x))

def leftorright_prob(left_v, right_v, lambda1, lambda2):
    return sigmoid(lambda1 * (1 / (right_v + lambda2) - 1 / (left_v + lambda2))) # the larger the left_v, the larger the prob return

class GridBusHumanRouterGreedy(BaseRouter):
    """RL cars will choose the shortest paths whenever possible
    """

    def choose_route(self, env):
        road_mode = ADDITIONAL_NET_PARAMS['road_mode']
        """See parent class."""
        speed_limit_main = ADDITIONAL_NET_PARAMS['speed_limit_main']
        speed_limit_extra = ADDITIONAL_NET_PARAMS['speed_limit_extra']
        speed_limit_sub = ADDITIONAL_NET_PARAMS['speed_limit_sub']
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0
        if road_mode == "without_central":
            if len(veh_next_edge) == no_next:
                next_route = None

            elif veh_edge == 'center_up':
                next_route = ['center_up','right_vertical']

            elif veh_edge == 'left_up':
                next_route = ['left_up','center_up','right_vertical'] # jump through the central

            elif veh_edge == 'inflow_up':
                # Greedy selection
                
                # Define left_up velocity
                left_up_vehs = env.k.vehicle.get_ids_by_edge('left_up')
                if len(left_up_vehs)>0:
                    sorted_left_up_vehs =sorted(left_up_vehs, key=env._get_abs_position)
                    # left_up_velocity = env.k.vehicle.get_speed(sorted_left_up_vehs[0])
                    left_up_velocity = sum(env.k.vehicle.get_speed(sorted_left_up_vehs)) / len(left_up_vehs)
                else:
                    left_up_velocity = speed_limit_main #maximum speed

                # Define left_bottom velocity
                left_bottom_vehs = env.k.vehicle.get_ids_by_edge('left_bottom')
                if len(left_bottom_vehs)>0:
                    sorted_left_bottom_vehs =sorted(left_bottom_vehs, key=env._get_abs_position)
                    # left_bottom_velocity = env.k.vehicle.get_speed(sorted_left_bottom_vehs[0])
                    left_bottom_velocity = sum(env.k.vehicle.get_speed(sorted_left_bottom_vehs))/ len(left_bottom_vehs)
                else:
                    left_bottom_velocity = speed_limit_main

                # Define left_vertical velocity
                left_vertical_vehs = env.k.vehicle.get_ids_by_edge('left_vertical')
                if len(left_vertical_vehs)>0:
                    sorted_left_vertical_vehs =sorted(left_vertical_vehs, key=env._get_abs_position)
                    # left_vertical_velocity = env.k.vehicle.get_speed(sorted_left_vertical_vehs[0])
                    left_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_left_vertical_vehs)) / len(left_vertical_vehs)
                else:
                    left_vertical_velocity = speed_limit_sub

                # Define center_vertical velocity
                center_vertical_vehs = env.k.vehicle.get_ids_by_edge('center_vertical')
                if len(center_vertical_vehs)>0:
                    sorted_center_vertical_vehs =sorted(center_vertical_vehs, key=env._get_abs_position)
                    # center_vertical_velocity = env.k.vehicle.get_speed(sorted_center_vertical_vehs[0])
                    center_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_center_vertical_vehs)) / len(center_vertical_vehs)
                else:
                    center_vertical_velocity = speed_limit_extra #maximum speed

                # Define center_bottom velocity
                center_bottom_vehs = env.k.vehicle.get_ids_by_edge('center_bottom')
                if len(center_bottom_vehs)>0:
                    sorted_center_bottom_vehs =sorted(center_bottom_vehs, key=env._get_abs_position)
                    center_bottom_velocity = env.k.vehicle.get_speed(sorted_center_bottom_vehs[0])
                    center_bottom_velocity = sum(env.k.vehicle.get_speed(sorted_center_bottom_vehs)) / len(center_bottom_vehs)
                else:
                    center_bottom_velocity = speed_limit_main #maximum speed

                # Define right_vertical velocity
                right_vertical_vehs = env.k.vehicle.get_ids_by_edge('right_vertical')
                if len(right_vertical_vehs)>0:
                    sorted_right_vertical_vehs =sorted(right_vertical_vehs, key=env._get_abs_position)
                    # right_vertical_velocity = env.k.vehicle.get_speed(sorted_right_vertical_vehs[0])
                    right_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_right_vertical_vehs)) / len(right_vertical_vehs)
                else:
                    right_vertical_velocity = speed_limit_sub #maximum speed

                # Define center_up velocity
                center_up_vehs = env.k.vehicle.get_ids_by_edge('center_up')
                if len(center_up_vehs)>0:
                    sorted_center_up_vehs =sorted(center_up_vehs, key=env._get_abs_position)  # The slowest speed
                    # center_up_velocity = env.k.vehicle.get_speed(sorted_center_up_vehs[0])
                    center_up_velocity = sum(env.k.vehicle.get_speed(sorted_center_up_vehs)) / len(center_up_vehs)
                else:
                    center_up_velocity = speed_limit_main # maximum speed

                up_velocity = (left_up_velocity + center_up_velocity + right_vertical_velocity) /3 # average speed
                down_velocity = (left_vertical_velocity + left_bottom_velocity + center_bottom_velocity) /3 # average speed
                up_queue = len(left_up_vehs) + len(center_up_vehs) + len(right_vertical_vehs)
                down_queue = len(left_vertical_vehs) + len(left_bottom_vehs) + len(center_bottom_vehs)
                # (np.percentile(outer_velocity, 80) > np.percentile(central_velocity, 80))
                # if  len(central_vehs) > 3 * len(top_vehs + right_vehs):

                # # up_prob = leftorright_prob(up_velocity, down_velocity, 10, 0.05)
                # up_prob = leftorright_prob(up_velocity/(up_queue+1), down_velocity/(down_queue+1), 10, 0.05)

                # if np.random.uniform(0,1,1)[0] <= up_prob:
                #     next_route = ['inflow_up','left_up'] # go straight
                # else:
                #     next_route = ['inflow_up','left_vertical'] #  go down
                if up_velocity > down_velocity:
                    next_route = ['inflow_up','left_up']
                elif up_velocity < down_velocity:
                    next_route = ['inflow_up','left_vertical']
                elif np.random.uniform(0,1,1)[0] <= 0.5:
                    next_route = ['inflow_up','left_up']
                else:
                    next_route = ['inflow_up','left_vertical']

            elif veh_edge == veh_route[-1]:
                num_routes = len(env.available_routes[veh_edge])
                frac = [val[1] for val in env.available_routes[veh_edge]]
                route_id = np.random.choice(
                    [i for i in range(num_routes)], size=1, p=frac)[0]

                next_route = env.available_routes[veh_edge][route_id][0]
            else:
                next_route = None

            return next_route

        elif road_mode == "without_right":
            if len(veh_next_edge) == no_next:
                next_route = None

            elif veh_edge == 'center_up':
                next_route = ['center_up','right_vertical']

            elif veh_edge == 'left_up':
                next_route = ['left_up','center_vertical'] # jump through the central

            elif veh_edge == 'inflow_up':
                # Greedy selection
                
                # Define left_up velocity
                left_up_vehs = env.k.vehicle.get_ids_by_edge('left_up')
                if len(left_up_vehs)>0:
                    sorted_left_up_vehs =sorted(left_up_vehs, key=env._get_abs_position)
                    # left_up_velocity = env.k.vehicle.get_speed(sorted_left_up_vehs[0])
                    left_up_velocity = sum(env.k.vehicle.get_speed(sorted_left_up_vehs)) / len(left_up_vehs)
                else:
                    left_up_velocity = speed_limit_main #maximum speed

                # Define left_bottom velocity
                left_bottom_vehs = env.k.vehicle.get_ids_by_edge('left_bottom')
                if len(left_bottom_vehs)>0:
                    sorted_left_bottom_vehs =sorted(left_bottom_vehs, key=env._get_abs_position)
                    # left_bottom_velocity = env.k.vehicle.get_speed(sorted_left_bottom_vehs[0])
                    left_bottom_velocity = sum(env.k.vehicle.get_speed(sorted_left_bottom_vehs))/ len(left_bottom_vehs)
                else:
                    left_bottom_velocity = speed_limit_main

                # Define left_vertical velocity
                left_vertical_vehs = env.k.vehicle.get_ids_by_edge('left_vertical')
                if len(left_vertical_vehs)>0:
                    sorted_left_vertical_vehs =sorted(left_vertical_vehs, key=env._get_abs_position)
                    # left_vertical_velocity = env.k.vehicle.get_speed(sorted_left_vertical_vehs[0])
                    left_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_left_vertical_vehs)) / len(left_vertical_vehs)
                else:
                    left_vertical_velocity = speed_limit_sub

                # Define center_vertical velocity
                center_vertical_vehs = env.k.vehicle.get_ids_by_edge('center_vertical')
                if len(center_vertical_vehs)>0:
                    sorted_center_vertical_vehs =sorted(center_vertical_vehs, key=env._get_abs_position)
                    # center_vertical_velocity = env.k.vehicle.get_speed(sorted_center_vertical_vehs[0])
                    center_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_center_vertical_vehs)) / len(center_vertical_vehs)
                else:
                    center_vertical_velocity = speed_limit_extra #maximum speed

                # Define center_bottom velocity
                center_bottom_vehs = env.k.vehicle.get_ids_by_edge('center_bottom')
                if len(center_bottom_vehs)>0:
                    sorted_center_bottom_vehs =sorted(center_bottom_vehs, key=env._get_abs_position)
                    center_bottom_velocity = env.k.vehicle.get_speed(sorted_center_bottom_vehs[0])
                    center_bottom_velocity = sum(env.k.vehicle.get_speed(sorted_center_bottom_vehs)) / len(center_bottom_vehs)
                else:
                    center_bottom_velocity = speed_limit_main #maximum speed

                # Define right_vertical velocity
                right_vertical_vehs = env.k.vehicle.get_ids_by_edge('right_vertical')
                if len(right_vertical_vehs)>0:
                    sorted_right_vertical_vehs =sorted(right_vertical_vehs, key=env._get_abs_position)
                    # right_vertical_velocity = env.k.vehicle.get_speed(sorted_right_vertical_vehs[0])
                    right_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_right_vertical_vehs)) / len(right_vertical_vehs)
                else:
                    right_vertical_velocity = speed_limit_sub # maximum speed

                # Define center_up velocity
                center_up_vehs = env.k.vehicle.get_ids_by_edge('center_up')
                if len(center_up_vehs)>0:
                    sorted_center_up_vehs =sorted(center_up_vehs, key=env._get_abs_position)  # The slowest speed
                    # center_up_velocity = env.k.vehicle.get_speed(sorted_center_up_vehs[0])
                    center_up_velocity = sum(env.k.vehicle.get_speed(sorted_center_up_vehs)) / len(center_up_vehs)
                else:
                    center_up_velocity = speed_limit_main # maximum speed

                up_velocity = (left_up_velocity + center_vertical_velocity + center_bottom_velocity) /3 # average speed
                down_velocity = (left_vertical_velocity + left_bottom_velocity + center_bottom_velocity) /3 # average speed
                up_queue = len(left_up_vehs) + len(center_up_vehs) + len(right_vertical_vehs)
                down_queue = len(left_vertical_vehs) + len(left_bottom_vehs) + len(center_bottom_vehs)
                # (np.percentile(outer_velocity, 80) > np.percentile(central_velocity, 80))
                # if  len(central_vehs) > 3 * len(top_vehs + right_vehs):

                # # up_prob = leftorright_prob(up_velocity, down_velocity, 10, 0.05)
                # up_prob = leftorright_prob(up_velocity/(up_queue+1), down_velocity/(down_queue+1), 10, 0.05)

                # if np.random.uniform(0,1,1)[0] <= up_prob:
                #     next_route = ['inflow_up','left_up'] # go straight
                # else:
                #     next_route = ['inflow_up','left_vertical'] #  go down
                if up_velocity > down_velocity:
                    next_route = ['inflow_up','left_up']
                elif up_velocity < down_velocity:
                    next_route = ['inflow_up','left_vertical']
                elif np.random.uniform(0,1,1)[0] <= 0.5:
                    next_route = ['inflow_up','left_up']
                else:
                    next_route = ['inflow_up','left_vertical']

            elif veh_edge == veh_route[-1]:
                num_routes = len(env.available_routes[veh_edge])
                frac = [val[1] for val in env.available_routes[veh_edge]]
                route_id = np.random.choice(
                    [i for i in range(num_routes)], size=1, p=frac)[0]

                next_route = env.available_routes[veh_edge][route_id][0]
            else:
                next_route = None

            return next_route

        elif road_mode == "allin":
            if len(veh_next_edge) == no_next:
                next_route = None

            elif veh_edge == 'center_up':
                next_route = ['center_up','right_vertical']

            elif veh_edge == 'left_up':
                # Greedy selection
                
                # Define center_vertical velocity
                center_vertical_vehs = env.k.vehicle.get_ids_by_edge('center_vertical')
                if len(center_vertical_vehs)>0:
                    sorted_center_vertical_vehs =sorted(center_vertical_vehs, key=env._get_abs_position)
                    # center_vertical_velocity = env.k.vehicle.get_speed(sorted_center_vertical_vehs[0])
                    center_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_center_vertical_vehs)) / len(center_vertical_vehs)
                else:
                    center_vertical_velocity = speed_limit_extra # maximum speed

                # Define center_bottom velocity
                center_bottom_vehs = env.k.vehicle.get_ids_by_edge('center_bottom')
                if len(center_bottom_vehs)>0:
                    sorted_center_bottom_vehs =sorted(center_bottom_vehs, key=env._get_abs_position)
                    # center_bottom_velocity = env.k.vehicle.get_speed(sorted_center_bottom_vehs[0])
                    center_bottom_velocity = sum(env.k.vehicle.get_speed(sorted_center_bottom_vehs)) / len(center_bottom_vehs)
                else:
                    center_bottom_velocity = speed_limit_main # maximum speed

                # Define right_vertical velocity
                right_vertical_vehs = env.k.vehicle.get_ids_by_edge('right_vertical')
                if len(right_vertical_vehs)>0:
                    sorted_right_vertical_vehs = sorted(right_vertical_vehs, key=env._get_abs_position)
                    # right_vertical_velocity = env.k.vehicle.get_speed(sorted_right_vertical_vehs[0])
                    right_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_right_vertical_vehs))/ len(right_vertical_vehs)
                else:
                    right_vertical_velocity = speed_limit_sub #maximum speed

                # Define center_up velocity
                center_up_vehs = env.k.vehicle.get_ids_by_edge('center_up')
                if len(center_up_vehs)>0:
                    sorted_center_up_vehs =sorted(center_up_vehs, key=env._get_abs_position)  # The clostest vehicle
                    # center_up_velocity = env.k.vehicle.get_speed(sorted_center_up_vehs[0])
                    center_up_velocity = sum(env.k.vehicle.get_speed(sorted_center_up_vehs)) / len(center_up_vehs)
                else:
                    center_up_velocity = speed_limit_main # maximum speed

                up_velocity = (center_up_velocity + right_vertical_velocity) /2 # average speed
                down_velocity = (center_bottom_velocity + center_vertical_velocity) /2 # average speed
                up_queue = len(center_up_vehs) + len(right_vertical_vehs)
                down_queue = len(center_bottom_vehs) + len(center_vertical_vehs)
                # (np.percentile(outer_velocity, 80) > np.percentile(central_velocity, 80))
                # if  len(central_vehs) > 3 * len(top_vehs + right_vehs):

                # mode based on probability selection
                # # up_prob = leftorright_prob(up_velocity, down_velocity, 100, 0.05)
                # up_prob = leftorright_prob(up_velocity/(up_queue+1), down_velocity/(down_queue+1), 100, 0.05)
                # # up_prob = 0.1
                # if np.random.uniform(0,1,1)[0] <= up_prob:
                #     next_route = ['left_up','center_up'] # go straight
                # else:
                #     next_route = ['left_up','center_vertical'] #  go down

                if up_velocity > down_velocity:
                    next_route = ['left_up','center_up']
                elif up_velocity < down_velocity:
                    next_route = ['left_up','center_vertical']
                elif np.random.uniform(0,1,1)[0] <= 0.5:
                    next_route = ['left_up','center_up']
                else:
                    next_route = ['left_up','center_vertical']

            elif veh_edge == 'inflow_up':
                # Greedy selection
                
                # Define left_up velocity
                left_up_vehs = env.k.vehicle.get_ids_by_edge('left_up')
                if len(left_up_vehs)>0:
                    sorted_left_up_vehs =sorted(left_up_vehs, key=env._get_abs_position)
                    # left_up_velocity = env.k.vehicle.get_speed(sorted_left_up_vehs[0])
                    left_up_velocity = sum(env.k.vehicle.get_speed(sorted_left_up_vehs)) / len(left_up_vehs)
                else:
                    left_up_velocity = speed_limit_main #maximum speed

                # Define left_bottom velocity
                left_bottom_vehs = env.k.vehicle.get_ids_by_edge('left_bottom')
                if len(left_bottom_vehs)>0:
                    sorted_left_bottom_vehs =sorted(left_bottom_vehs, key=env._get_abs_position)
                    # left_bottom_velocity = env.k.vehicle.get_speed(sorted_left_bottom_vehs[0])
                    left_bottom_velocity = sum(env.k.vehicle.get_speed(sorted_left_bottom_vehs))/ len(left_bottom_vehs)
                else:
                    left_bottom_velocity = speed_limit_main

                # Define left_vertical velocity
                left_vertical_vehs = env.k.vehicle.get_ids_by_edge('left_vertical')
                if len(left_vertical_vehs)>0:
                    sorted_left_vertical_vehs =sorted(left_vertical_vehs, key=env._get_abs_position)
                    # left_vertical_velocity = env.k.vehicle.get_speed(sorted_left_vertical_vehs[0])
                    left_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_left_vertical_vehs)) / len(left_vertical_vehs)
                else:
                    left_vertical_velocity = speed_limit_sub

                # Define center_vertical velocity
                center_vertical_vehs = env.k.vehicle.get_ids_by_edge('center_vertical')
                if len(center_vertical_vehs)>0:
                    sorted_center_vertical_vehs =sorted(center_vertical_vehs, key=env._get_abs_position)
                    # center_vertical_velocity = env.k.vehicle.get_speed(sorted_center_vertical_vehs[0])
                    center_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_center_vertical_vehs)) / len(center_vertical_vehs)
                else:
                    center_vertical_velocity = speed_limit_extra #maximum speed

                # Define center_bottom velocity
                center_bottom_vehs = env.k.vehicle.get_ids_by_edge('center_bottom')
                if len(center_bottom_vehs)>0:
                    sorted_center_bottom_vehs =sorted(center_bottom_vehs, key=env._get_abs_position)
                    center_bottom_velocity = env.k.vehicle.get_speed(sorted_center_bottom_vehs[0])
                    center_bottom_velocity = sum(env.k.vehicle.get_speed(sorted_center_bottom_vehs)) / len(center_bottom_vehs)
                else:
                    center_bottom_velocity = speed_limit_main #maximum speed

                # Define right_vertical velocity
                right_vertical_vehs = env.k.vehicle.get_ids_by_edge('right_vertical')
                if len(right_vertical_vehs)>0:
                    sorted_right_vertical_vehs =sorted(right_vertical_vehs, key=env._get_abs_position)
                    # right_vertical_velocity = env.k.vehicle.get_speed(sorted_right_vertical_vehs[0])
                    right_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_right_vertical_vehs)) / len(right_vertical_vehs)
                else:
                    right_vertical_velocity = speed_limit_sub #maximum speed

                # Define center_up velocity
                center_up_vehs = env.k.vehicle.get_ids_by_edge('center_up')
                if len(center_up_vehs)>0:
                    sorted_center_up_vehs =sorted(center_up_vehs, key=env._get_abs_position)  # The slowest speed
                    # center_up_velocity = env.k.vehicle.get_speed(sorted_center_up_vehs[0])
                    center_up_velocity = sum(env.k.vehicle.get_speed(sorted_center_up_vehs)) / len(center_up_vehs)
                else:
                    center_up_velocity = speed_limit_main # maximum speed

                up_velocity = (left_up_velocity + max(center_up_velocity + right_vertical_velocity, center_vertical_velocity+center_bottom_velocity)) /3 # average speed
                down_velocity = (left_vertical_velocity + left_bottom_velocity + center_bottom_velocity) /3 # average speed
                up_queue = len(left_up_vehs) + max(len(center_up_vehs) + len(right_vertical_vehs), len(center_vertical_vehs) + len(center_bottom_vehs))
                down_queue = len(left_vertical_vehs) + len(left_bottom_vehs) + len(center_bottom_vehs)
                # (np.percentile(outer_velocity, 80) > np.percentile(central_velocity, 80))
                # if  len(central_vehs) > 3 * len(top_vehs + right_vehs):

                # # up_prob = leftorright_prob(up_velocity, down_velocity, 10, 0.05)
                # up_prob = leftorright_prob(up_velocity/(up_queue+1), down_velocity/(down_queue+1), 10, 0.05)

                # if np.random.uniform(0,1,1)[0] <= up_prob:
                #     next_route = ['inflow_up','left_up'] # go straight
                # else:
                #     next_route = ['inflow_up','left_vertical'] #  go down
                if up_velocity > down_velocity:
                    next_route = ['inflow_up','left_up']
                elif up_velocity < down_velocity:
                    next_route = ['inflow_up','left_vertical']
                elif np.random.uniform(0,1,1)[0] <= 0.5:
                    next_route = ['inflow_up','left_up']
                else:
                    next_route = ['inflow_up','left_vertical']

            elif veh_edge == veh_route[-1]:
                num_routes = len(env.available_routes[veh_edge])
                frac = [val[1] for val in env.available_routes[veh_edge]]
                route_id = np.random.choice(
                    [i for i in range(num_routes)], size=1, p=frac)[0]

                next_route = env.available_routes[veh_edge][route_id][0]
            else:
                next_route = None

            return next_route

        elif road_mode == "without_left":
            if len(veh_next_edge) == no_next:
                next_route = None

            elif veh_edge == 'center_up':
                next_route = ['center_up','right_vertical']

            elif veh_edge == 'left_up':
                # Greedy selection
                
                # Define center_vertical velocity
                center_vertical_vehs = env.k.vehicle.get_ids_by_edge('center_vertical')
                if len(center_vertical_vehs)>0:
                    sorted_center_vertical_vehs =sorted(center_vertical_vehs, key=env._get_abs_position)
                    # center_vertical_velocity = env.k.vehicle.get_speed(sorted_center_vertical_vehs[0])
                    center_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_center_vertical_vehs)) / len(center_vertical_vehs)
                else:
                    center_vertical_velocity = speed_limit_extra # maximum speed

                # Define center_bottom velocity
                center_bottom_vehs = env.k.vehicle.get_ids_by_edge('center_bottom')
                if len(center_bottom_vehs)>0:
                    sorted_center_bottom_vehs =sorted(center_bottom_vehs, key=env._get_abs_position)
                    # center_bottom_velocity = env.k.vehicle.get_speed(sorted_center_bottom_vehs[0])
                    center_bottom_velocity = sum(env.k.vehicle.get_speed(sorted_center_bottom_vehs)) / len(center_bottom_vehs)
                else:
                    center_bottom_velocity = speed_limit_main # maximum speed

                # Define right_vertical velocity
                right_vertical_vehs = env.k.vehicle.get_ids_by_edge('right_vertical')
                if len(right_vertical_vehs)>0:
                    sorted_right_vertical_vehs = sorted(right_vertical_vehs, key=env._get_abs_position)
                    # right_vertical_velocity = env.k.vehicle.get_speed(sorted_right_vertical_vehs[0])
                    right_vertical_velocity = sum(env.k.vehicle.get_speed(sorted_right_vertical_vehs))/ len(right_vertical_vehs)
                else:
                    right_vertical_velocity = speed_limit_sub #maximum speed

                # Define center_up velocity
                center_up_vehs = env.k.vehicle.get_ids_by_edge('center_up')
                if len(center_up_vehs)>0:
                    sorted_center_up_vehs =sorted(center_up_vehs, key=env._get_abs_position)  # The clostest vehicle
                    # center_up_velocity = env.k.vehicle.get_speed(sorted_center_up_vehs[0])
                    center_up_velocity = sum(env.k.vehicle.get_speed(sorted_center_up_vehs)) / len(center_up_vehs)
                else:
                    center_up_velocity = speed_limit_main # maximum speed

                up_velocity = (center_up_velocity + right_vertical_velocity) /2 # average speed
                down_velocity = (center_bottom_velocity + center_vertical_velocity) /2 # average speed
                up_queue = len(center_up_vehs) + len(right_vertical_vehs)
                down_queue = len(center_bottom_vehs) + len(center_vertical_vehs)
                # (np.percentile(outer_velocity, 80) > np.percentile(central_velocity, 80))
                # if  len(central_vehs) > 3 * len(top_vehs + right_vehs):

                # mode based on probability selection
                # # up_prob = leftorright_prob(up_velocity, down_velocity, 100, 0.05)
                # up_prob = leftorright_prob(up_velocity/(up_queue+1), down_velocity/(down_queue+1), 100, 0.05)
                # # up_prob = 0.1
                # if np.random.uniform(0,1,1)[0] <= up_prob:
                #     next_route = ['left_up','center_up'] # go straight
                # else:
                #     next_route = ['left_up','center_vertical'] #  go down

                if up_velocity > down_velocity:
                    next_route = ['left_up','center_up']
                elif up_velocity < down_velocity:
                    next_route = ['left_up','center_vertical']
                elif np.random.uniform(0,1,1)[0] <= 0.5:
                    next_route = ['left_up','center_up']
                else:
                    next_route = ['left_up','center_vertical']

            elif veh_edge == 'inflow_up':
                # Greedy selection
                next_route = ['inflow_up','left_up']

            elif veh_edge == veh_route[-1]:
                num_routes = len(env.available_routes[veh_edge])
                frac = [val[1] for val in env.available_routes[veh_edge]]
                route_id = np.random.choice(
                    [i for i in range(num_routes)], size=1, p=frac)[0]

                next_route = env.available_routes[veh_edge][route_id][0]
            else:
                next_route = None

            return next_route


class RingBusHumanRouter20SitAware(BaseRouter):
    """RL cars will choose the shortest paths whenever possible
    """

    def choose_route(self, env):
        """See parent class."""
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0
        if len(veh_next_edge) == no_next:
            next_route = None
        elif veh_edge == 'right':
            #Greedy selection
            
            # Define central velocity
            central_vehs = env.k.vehicle.get_ids_by_edge('central')
            top_vehs = env.k.vehicle.get_ids_by_edge('top')
            right_vehs = env.k.vehicle.get_ids_by_edge('right') + env.k.vehicle.get_ids_by_edge(':right_0')
            right_pos = 10001
            if len(right_vehs)>0:
                sorted_right_vehs =sorted(right_vehs, key=env._get_abs_position)
                # right_velocity = env.k.vehicle.get_speed(sorted_right_vehs[0])
                right_avg_vel = np.mean(env.k.vehicle.get_speed(right_vehs))
                right_pos = env.k.vehicle.get_position(sorted_right_vehs[0])
            else:
                right_velocity = 30 #maximum speed
                right_avg_vel = 30

            central_pos = 10001
            if len(central_vehs)>0:
                sorted_central_vehs =sorted(central_vehs, key=env._get_abs_position)
                # central_velocity = env.k.vehicle.get_speed(sorted_central_vehs[0])
                central_avg_vel = np.mean(env.k.vehicle.get_speed(central_vehs))
                central_pos = env.k.vehicle.get_position(sorted_central_vehs[0])
                # print(f'central: pos {env.k.vehicle.get_position(sorted_central_vehs[0])}, speed {central_velocity}')
            else:
                central_velocity = 30 #maximum speed
                central_avg_vel = 30

            prob = 0.2
            
            # print(f'right velocity: {right_velocity}, central velocity: {central_velocity}')
            if np.random.uniform(0,1,1)[0] <= prob:
                # if (len(top_vehs) + len(right_vehs) < tolerance * len(central_vehs)) and not (leader_speed <2 and leader_next_edge == 'right'):
                if (right_avg_vel > central_avg_vel and right_pos > central_pos) and (len(top_vehs) + len(right_vehs) < len(central_vehs)):
                    next_route = ['right','top'] # do not go central
                else:
                    next_route = ['right','central']
            else:
                next_route = ['right','central'] #  go central 
        elif veh_edge == veh_route[-1]:

            num_routes = len(env.available_routes[veh_edge])
            frac = [val[1] for val in env.available_routes[veh_edge]]
            route_id = np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0]

            next_route = env.available_routes[veh_edge][route_id][0]
        else:
            next_route = None

        return next_route
    
class RingBusHumanRouter100SitAware(BaseRouter):
    """RL cars will choose the shortest paths whenever possible
    """

    def choose_route(self, env):
        """See parent class."""
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0
        if len(veh_next_edge) == no_next:
            next_route = None
        elif veh_edge == 'right':
            #Greedy selection
            
            # Define central velocity
            central_vehs = env.k.vehicle.get_ids_by_edge('central')
            top_vehs = env.k.vehicle.get_ids_by_edge('top')
            right_vehs = env.k.vehicle.get_ids_by_edge('right') + env.k.vehicle.get_ids_by_edge(':right_0')
            right_pos = 10001
            if len(right_vehs)>0:
                sorted_right_vehs =sorted(right_vehs, key=env._get_abs_position)
                # right_velocity = env.k.vehicle.get_speed(sorted_right_vehs[0])
                right_avg_vel = np.mean(env.k.vehicle.get_speed(right_vehs))
                right_pos = env.k.vehicle.get_position(sorted_right_vehs[0])
            else:
                right_velocity = 30 #maximum speed
                right_avg_vel = 30

            central_pos = 10001
            if len(central_vehs)>0:
                sorted_central_vehs =sorted(central_vehs, key=env._get_abs_position)
                # central_velocity = env.k.vehicle.get_speed(sorted_central_vehs[0])
                central_avg_vel = np.mean(env.k.vehicle.get_speed(central_vehs))
                central_pos = env.k.vehicle.get_position(sorted_central_vehs[0])
                # print(f'central: pos {env.k.vehicle.get_position(sorted_central_vehs[0])}, speed {central_velocity}')
            else:
                central_velocity = 30 #maximum speed
                central_avg_vel = 30

            if len(top_vehs)>0:
                sorted_top_vehs =sorted(top_vehs, key=env._get_abs_position)
                # central_velocity = env.k.vehicle.get_speed(sorted_central_vehs[0])
                top_avg_vel = np.mean(env.k.vehicle.get_speed(top_vehs))
                # central_pos = env.k.vehicle.get_position(sorted_central_vehs[0])
                # print(f'central: pos {env.k.vehicle.get_position(sorted_central_vehs[0])}, speed {central_velocity}')
            else:
                # central_velocity = 30 #maximum speed
                top_avg_vel = 30

            prob = 1
            # print(f'top: {top_avg_vel}, right: {right_avg_vel}, central: {central_avg_vel}')
            # print(f'right velocity: {right_velocity}, central velocity: {central_velocity}')
            if np.random.uniform(0,1,1)[0] <= prob:
                # if (len(top_vehs) + len(right_vehs) < tolerance * len(central_vehs)) and not (leader_speed <2 and leader_next_edge == 'right'):
                # if (right_avg_vel > central_avg_vel and right_pos > central_pos) and (len(top_vehs) + len(right_vehs) < len(central_vehs)):
                # if (top_avg_vel > central_avg_vel ) & (right_avg_vel > central_avg_vel) & ((len(top_vehs) + len(right_vehs)) < len(central_vehs)):
                if (top_avg_vel > central_avg_vel ) & (right_avg_vel > central_avg_vel):
                    next_route = ['right','top'] # do not go central
                else:
                    next_route = ['right','central']
            else:
                next_route = ['right','central'] #  go central 
        elif veh_edge == veh_route[-1]:
            
            num_routes = len(env.available_routes[veh_edge])
            frac = [val[1] for val in env.available_routes[veh_edge]]
            route_id = np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0]

            next_route = env.available_routes[veh_edge][route_id][0]
        else:
            next_route = None

        return next_route

class RingBusHumanRouterCentralQueue(BaseRouter):
    """RL cars will choose the shortest paths whenever possible
    """

    def choose_route(self, env):
        """See parent class."""
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0
        if len(veh_next_edge) == no_next:
            next_route = None
        elif veh_edge == 'bottom':
            #Greedy selection
            
            # Define central velocity
            central_vehs = env.k.vehicle.get_ids_by_edge('central')
            top_vehs = env.k.vehicle.get_ids_by_edge('top')
            right_vehs = env.k.vehicle.get_ids_by_edge('right') + env.k.vehicle.get_ids_by_edge(':right_0')
            right_pos = 10001
            if len(right_vehs)>0:
                sorted_right_vehs =sorted(right_vehs, key=env._get_abs_position)
                # right_velocity = env.k.vehicle.get_speed(sorted_right_vehs[0])
                right_avg_vel = np.mean(env.k.vehicle.get_speed(right_vehs))
                right_pos = env.k.vehicle.get_position(sorted_right_vehs[0])
            else:
                right_velocity = 30 #maximum speed
                right_avg_vel = 30

            central_pos = 10001
            if len(central_vehs)>0:
                sorted_central_vehs =sorted(central_vehs, key=env._get_abs_position)
                # central_velocity = env.k.vehicle.get_speed(sorted_central_vehs[0])
                central_avg_vel = np.mean(env.k.vehicle.get_speed(central_vehs))
                central_pos = env.k.vehicle.get_position(sorted_central_vehs[0])
                # print(f'central: pos {env.k.vehicle.get_position(sorted_central_vehs[0])}, speed {central_velocity}')
            else:
                central_velocity = 30 #maximum speed
                central_avg_vel = 30

            prob = 1
            
            # print(f'right velocity: {right_velocity}, central velocity: {central_velocity}')
            if np.random.uniform(0,1,1)[0] <= prob:
                # if (len(top_vehs) + len(right_vehs) < tolerance * len(central_vehs)) and not (leader_speed <2 and leader_next_edge == 'right'):
                if (right_avg_vel > central_avg_vel and right_pos > central_pos) and (len(top_vehs) + len(right_vehs) < len(central_vehs)):
                    next_route = ['bottom','right'] # do not go central
                else:
                    next_route = ['bottom','central']
            else:
                next_route = ['bottom','central'] #  go central 
        elif veh_edge == veh_route[-1]:
            
            num_routes = len(env.available_routes[veh_edge])
            frac = [val[1] for val in env.available_routes[veh_edge]]
            route_id = np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0]

            next_route = env.available_routes[veh_edge][route_id][0]
        else:
            next_route = None

        return next_route

class RingBusRouter(BaseRouter):
    """RL cars will choose the shortest paths whenever possible
    """

    def choose_route(self, env):
        """See parent class."""
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0

        if len(veh_next_edge) == no_next:
            next_route = None
        elif veh_edge == 'right':
            # next_route = ['right','central'] # always go central 
            next_route = ['right','top'] # do not go central

            # random_route = random.randint(0, len(veh_next_edge) - 1)
            # while veh_next_edge[0][0][0] == not_an_edge:
            #     veh_next_edge = env.k.network.next_edge(
            #         veh_next_edge[random_route][0],
            #         veh_next_edge[random_route][1])
            # next_route = [veh_edge, veh_next_edge[0][0]]
            # print(veh_route,veh_edge,veh_next_edge,random_route,next_route)
        elif veh_edge == veh_route[-1]:
            # edge = env.k.vehicle.get_edge(self.veh_id)            
            # current_route = env.k.vehicle.get_route(self.veh_id)
            # print(veh_route,veh_edge,veh_next_edge)
            # choose one of the available routes based on the fraction of times
            # the given route can be chosen
            # num_routes = len(env.available_routes[edge])
            # frac = [val[1] for val in env.available_routes[edge]]
            # route_id = np.random.choice(
            #     [i for i in range(num_routes)], size=1, p=frac)[0]

            # pass the chosen route
            
            num_routes = len(env.available_routes[veh_edge])
            frac = [val[1] for val in env.available_routes[veh_edge]]
            route_id = np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0]

            next_route = env.available_routes[veh_edge][route_id][0]
        else:
            next_route = None

        return next_route

class RingBusRouter_Two(BaseRouter):
    """RL cars will choose the shortest paths whenever possible
    """

    def choose_route(self, env):
        """See parent class."""
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0

        if len(veh_next_edge) == no_next:
            next_route = None
        elif veh_edge == 'right':
            next_route = ['right','central'] # always go central 
            # next_route = ['right','top'] # do not go central

            # random_route = random.randint(0, len(veh_next_edge) - 1)
            # while veh_next_edge[0][0][0] == not_an_edge:
            #     veh_next_edge = env.k.network.next_edge(
            #         veh_next_edge[random_route][0],
            #         veh_next_edge[random_route][1])
            # next_route = [veh_edge, veh_next_edge[0][0]]
            # print(veh_route,veh_edge,veh_next_edge,random_route,next_route)
        elif veh_edge == veh_route[-1]:
            # edge = env.k.vehicle.get_edge(self.veh_id)            
            # current_route = env.k.vehicle.get_route(self.veh_id)
            # print(veh_route,veh_edge,veh_next_edge)
            # choose one of the available routes based on the fraction of times
            # the given route can be chosen
            # num_routes = len(env.available_routes[edge])
            # frac = [val[1] for val in env.available_routes[edge]]
            # route_id = np.random.choice(
            #     [i for i in range(num_routes)], size=1, p=frac)[0]

            # pass the chosen route
            
            num_routes = len(env.available_routes[veh_edge])
            frac = [val[1] for val in env.available_routes[veh_edge]]
            route_id = np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0]

            next_route = env.available_routes[veh_edge][route_id][0]
        else:
            next_route = None

        return next_route

class GridRouter(BaseRouter):
    """A router used to re-route a vehicle in a traffic light grid environment.

    Usage
    -----
    See base class for usage example.
    """

    def choose_route(self, env):
        """See parent class."""
        if len(env.k.vehicle.get_route(self.veh_id)) == 0:
            # this occurs to inflowing vehicles, whose information is not added
            # to the subscriptions in the first step that they departed
            return None
        elif env.k.vehicle.get_edge(self.veh_id) == \
                env.k.vehicle.get_route(self.veh_id)[-1]:
            return [env.k.vehicle.get_edge(self.veh_id)]
        else:
            return None


class BayBridgeRouter(ContinuousRouter):
    """Assists in choosing routes in select cases for the Bay Bridge network.

    Extension to the Continuous Router.

    Usage
    -----
    See base class for usage example.
    """

    def choose_route(self, env):
        """See parent class."""
        edge = env.k.vehicle.get_edge(self.veh_id)
        lane = env.k.vehicle.get_lane(self.veh_id)

        if edge == "183343422" and lane in [2] \
                or edge == "124952179" and lane in [1, 2]:
            new_route = env.available_routes[edge + "_1"][0][0]
        else:
            new_route = super().choose_route(env)

        return new_route


class I210Router(ContinuousRouter):
    """Assists in choosing routes in select cases for the I-210 sub-network.

    Extension to the Continuous Router.

    Usage
    -----
    See base class for usage example.
    """

    def choose_route(self, env):
        """See parent class."""
        edge = env.k.vehicle.get_edge(self.veh_id)
        lane = env.k.vehicle.get_lane(self.veh_id)

        # vehicles on these edges in lanes 4 and 5 are not going to be able to
        # make it out in time
        if edge == "119257908#1-AddedOffRampEdge" and lane in [5, 4, 3]:
            new_route = env.available_routes[
                "119257908#1-AddedOffRampEdge"][0][0]
        else:
            new_route = super().choose_route(env)

        return new_route
