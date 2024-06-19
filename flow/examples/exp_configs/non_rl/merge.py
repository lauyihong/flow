"""Example of a merge network with human-driven vehicles.

In the absence of autonomous vehicles, the network exhibits properties of
convective instability, with perturbations propagating upstream from the merge
point before exiting the network.
"""

from flow.core.params import SumoParams, EnvParams, \
    NetParams, InitialConfig, InFlows, SumoCarFollowingParams
from flow.core.params import VehicleParams
from flow.controllers import IDMController,BaseLaneChangeController, MinicityRouter, GridBusHumanRouterGreedy
# from flow.envs.merge import MergePOEnv, ADDITIONAL_ENV_PARAMS
from flow.envs.ring.accel import AccelEnv, ADDITIONAL_ENV_PARAMS
from flow.networks import MergeNetwork
from flow.networks import GridMergeNetwork
from flow.networks.gridmerge import ADDITIONAL_NET_PARAMS
# inflow rate at the highway
FLOW_RATE = ADDITIONAL_NET_PARAMS['flow_rate']
# FLOW_RATE = 1500
# class CentralLaneController(BaseLaneChangeController):
#     """A lane-changing model used to move vehicles into lane 0."""
#     ##### Below this is new code #####
#     def get_lane_change_action(self, env):
#         current_lane = env.k.vehicle.get_lane(self.veh_id)
#         if current_lane > 0:
#             return -1 # move to the right
#         else:
#             return 0 # do not move

vehicles = VehicleParams()
vehicles.add(
    veh_id="human",
    acceleration_controller=(IDMController, {
        "noise": 3 # 10
    }),
    car_following_params=SumoCarFollowingParams(
        max_speed = 50,
        min_gap = 5.0,
        impatience = 0.9,
        speed_mode="right_of_way", #  obey_safe_speed
    ),
    # lane_change_controller=(CentralLaneController, {}),
    routing_controller=(GridBusHumanRouterGreedy, {}), # MinicityRouter, continuous to choose the next route after the current route
    num_vehicles = 5)

inflow = InFlows()
# for grid network simulation
inflow.add(
    veh_type="human",
    edge="inflow_up",
    vehs_per_hour=FLOW_RATE,
    depart_lane="free",
    departSpeed=7.5)

# inflow.add(
#     veh_type="human",
#     edge="inflow_bottom",
#     vehs_per_hour=1000,
#     departLane="free",
#     departSpeed=7.5)

# # for merge network simulation
# inflow.add(
#     veh_type="human",
#     edge="inflow_highway",
#     vehs_per_hour=FLOW_RATE,
#     departLane="free",
#     departSpeed=10)
# inflow.add(
#     veh_type="human",
#     edge="inflow_merge",
#     vehs_per_hour=1000,
#     departLane="free",
#     departSpeed=7.5)


flow_params = dict(
    # name of the experiment
    exp_tag='grid-merge-baseline',

    # name of the flow environment the experiment is running on
    env_name=AccelEnv,

    # name of the network class the experiment is running on
    network=GridMergeNetwork,
    # network=MergeNetwork,

    # simulator that is used by the experiment
    simulator='traci',

    # sumo-related parameters (see flow.core.params.SumoParams)
    sim=SumoParams(
        render=True,
        emission_path="./data/",
        sim_step=0.2,
        restart_instance=False,
    ),

    # environment related parameters (see flow.core.params.EnvParams)
    env=EnvParams(
        horizon=800,
        additional_params=ADDITIONAL_ENV_PARAMS,
        sims_per_step=5,
        warmup_steps=200,
    ),

    # network-related parameters (see flow.core.params.NetParams and the
    # network's documentation or ADDITIONAL_NET_PARAMS component)
    net=NetParams(
        inflows=inflow,
        additional_params= ADDITIONAL_NET_PARAMS, 
        # {
        #     # length of the unit length 
        #     "unit_length": 100,
        #     # length or the inflow edge
        #     "inflow_edge_len": 100,
        #     # number of lanes in the main road
        #     "main_lanes": 1,
        #     # number of lanes in the sub road
        #     "sub_lanes": 1,
        #     # max speed limit of the network
        #     "speed_limit_main": 30,
        #     # max speed limit of the sub road
        #     "speed_limit_sub": 30,

        #     # for merge network simulation
        #     "merge_length": 100,
        #     "pre_merge_length": 500,
        #     "post_merge_length": 100,
        #     "merge_lanes": 1,
        #     "highway_lanes": 1,
        #     "speed_limit": 30,
        # },
    ),

    # vehicles to be placed in the network at the start of a rollout (see
    # flow.core.params.VehicleParams)
    veh=vehicles,

    # parameters specifying the positioning of vehicles upon initialization/
    # reset (see flow.core.params.InitialConfig)
    initial=InitialConfig(
        spacing="uniform",
        perturbation=5.0,
    ),
)
