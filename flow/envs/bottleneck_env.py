from flow.envs.base_env import Env
from flow.envs.loop_accel import AccelEnv
from flow.core import rewards
from flow.core import multi_agent_rewards

from gym.spaces.box import Box
from gym.spaces.tuple_space import Tuple
from collections import defaultdict

import numpy as np

EDGE_LIST = ["1", "2", "3", "4", "5"]
EDGE_BEFORE_TOLL = "1"
TB_TL_ID = "2"
EDGE_AFTER_TOLL = "2"
NUM_TOLL_LANES = 16
TOLL_BOOTH_AREA = 10 # how far into the edge lane changing is disabled
RED_LIGHT_DIST = 50 # controls how close we have to be for the red light to start going off

EDGE_BEFORE_RAMP_METER = "2"
EDGE_AFTER_RAMP_METER = "3"
NUM_RAMP_METERS = 14
RAMP_METER_AREA = 80

MAX_LANES = 16

MEAN_NUM_SECONDS_WAIT_AT_FAST_TRACK = 3
MEAN_NUM_SECONDS_WAIT_AT_TOLL = 15
FAST_TRACK_ON = range(6, 11)

class BottleneckEnv(AccelEnv):
    def __init__(self, env_params, sumo_params, scenario):
        self.num_rl = scenario.vehicles.num_rl_vehicles
        super().__init__(env_params, sumo_params, scenario)
        self.edge_dict = defaultdict(list)
        self.cars_waiting_for_toll = dict()
        self.cars_waiting_before_ramp_meter = dict()
        self.toll_wait_time = np.abs(
            np.random.normal(MEAN_NUM_SECONDS_WAIT_AT_TOLL / self.sim_step, 4 / self.sim_step, NUM_TOLL_LANES))
        self.tl_state = ""
        self.disable_tb = False
        self.disable_ramp_metering = False

        print(env_params.additional_params)
        if "disable_tb" in env_params.additional_params:
            self.disable_tb = env_params.get_additional_param("disable_tb")

        if "disable_ramp_metering" in env_params.additional_params:
            self.disable_ramp_metering = env_params.get_additional_param("disable_ramp_metering")

        print(self.disable_tb)


    def additional_command(self):
        super().additional_command()
        # build a list of vehicles and their edges and positions
        self.edge_dict = defaultdict(list)
        # update the dict with all the edges in edge_list so we can look forward for edges
        self.edge_dict.update((k, [[] for _ in range(MAX_LANES)]) for k in EDGE_LIST)
        for veh_id in self.vehicles.get_ids():
            edge = self.vehicles.get_edge(veh_id)
            if edge not in self.edge_dict:
                self.edge_dict.update({edge: [[] for _ in range(MAX_LANES)]})
            lane = self.vehicles.get_lane(veh_id)  # integer
            pos = self.vehicles.get_position(veh_id)
            self.edge_dict[edge][lane].append((veh_id, pos))
        if not self.disable_tb:
            self.apply_toll_bridge_control()
        if not self.disable_ramp_metering:
            self.ramp_meter_lane_change_control()


    def ramp_meter_lane_change_control(self):
        cars_that_have_left = []
        for veh_id in self.cars_waiting_before_ramp_meter:
            if self.vehicles.get_edge(veh_id) == EDGE_AFTER_RAMP_METER:
                lane_change_mode = self.cars_waiting_before_ramp_meter[veh_id]["lane_change_mode"]
                color = self.cars_waiting_before_ramp_meter[veh_id]["color"]
                self.traci_connection.vehicle.setColor(veh_id, color)
                self.traci_connection.vehicle.setLaneChangeMode(veh_id, lane_change_mode)

                cars_that_have_left.append(veh_id)

        for veh_id in cars_that_have_left:
            self.cars_waiting_before_ramp_meter.__delitem__(veh_id)

        for lane in range(NUM_RAMP_METERS):
            cars_in_lane = self.edge_dict[EDGE_BEFORE_RAMP_METER][lane]

            for car in cars_in_lane:
                veh_id, pos = car
                if pos > RAMP_METER_AREA:
                    if veh_id not in self.cars_waiting_for_toll:
                        # Disable lane changes inside Toll Area
                        lane_change_mode = self.vehicles.get_lane_change_mode(veh_id)
                        color = self.traci_connection.vehicle.getColor(veh_id)
                        self.cars_waiting_before_ramp_meter[veh_id] = {"lane_change_mode": lane_change_mode, "color": color}
                        self.traci_connection.vehicle.setLaneChangeMode(veh_id, 512)
                        self.traci_connection.vehicle.setColor(veh_id, (0, 255, 255, 0))


    def apply_toll_bridge_control(self):
        cars_that_have_left = []
        for veh_id in self.cars_waiting_for_toll:
            if self.vehicles.get_edge(veh_id) == EDGE_AFTER_TOLL:
                lane = self.vehicles.get_lane(veh_id)
                lane_change_mode = self.cars_waiting_for_toll[veh_id]["lane_change_mode"]
                color = self.cars_waiting_for_toll[veh_id]["color"]
                self.traci_connection.vehicle.setColor(veh_id, color)
                self.traci_connection.vehicle.setLaneChangeMode(veh_id, lane_change_mode)
                if lane not in FAST_TRACK_ON:
                    self.toll_wait_time[lane] = max(0,
                                                    np.random.normal(MEAN_NUM_SECONDS_WAIT_AT_TOLL / self.sim_step,
                                                                     1 / self.sim_step))
                else:
                    self.toll_wait_time[lane] = max(0,
                                                    np.random.normal(MEAN_NUM_SECONDS_WAIT_AT_FAST_TRACK / self.sim_step,
                                                                     1 / self.sim_step))

                cars_that_have_left.append(veh_id)

        for veh_id in cars_that_have_left:
            self.cars_waiting_for_toll.__delitem__(veh_id)

        traffic_light_states = ["G"] * NUM_TOLL_LANES

        for lane in range(NUM_TOLL_LANES):
            cars_in_lane = self.edge_dict[EDGE_BEFORE_TOLL][lane]

            for car in cars_in_lane:
                veh_id, pos = car
                if pos > TOLL_BOOTH_AREA:
                    if veh_id not in self.cars_waiting_for_toll:
                        # Disable lane changes inside Toll Area
                        lane_change_mode = self.vehicles.get_lane_change_mode(veh_id)
                        color = self.traci_connection.vehicle.getColor(veh_id)
                        self.cars_waiting_for_toll[veh_id] = {"lane_change_mode": lane_change_mode, "color": color}
                        self.traci_connection.vehicle.setLaneChangeMode(veh_id, 512)
                        self.traci_connection.vehicle.setColor(veh_id, (255, 0, 255, 0))
                    else:
                        if pos > 50:
                            if self.toll_wait_time[lane] < 0:
                                traffic_light_states[lane] = "G"
                            else:
                                traffic_light_states[lane] = "r"
                                self.toll_wait_time[lane] -= 1

        newTLState = "".join(traffic_light_states)

        if newTLState != self.tl_state:
            self.tl_state = newTLState
            self.traci_connection.trafficlights.setRedYellowGreenState(tlsID=TB_TL_ID, state=newTLState)