"""Used as an example of ring experiment.

This example consists of 22 IDM cars on a ring creating shockwaves.
"""

from flow.controllers import IDMController, ContinuousRouter,BaseLaneChangeController,SimLaneChangeController,MinicityRouter, \
    RingBusRouter,RingBusRouter_Two,RingBusHumanRouterGreedy,RingBusHumanRouter100SitAware,RingBusHumanRouter20SitAware

from flow.core.params import SumoParams, EnvParams, InitialConfig, NetParams
from flow.core.params import VehicleParams
from flow.envs.ring.accel import AccelEnv, ADDITIONAL_ENV_PARAMS
from flow.networks.ring_cross import RingCrossNetwork, ADDITIONAL_NET_PARAMS

class CentralLaneController(BaseLaneChangeController):
    """A lane-changing model used to move vehicles into lane 0."""
    ##### Below this is new code #####
    def get_lane_change_action(self, env):
        current_lane = env.k.vehicle.get_lane(self.veh_id)
        if current_lane > 0:
            return -1
        else:
            return 0

vehicles = VehicleParams()
vehicles.add(
    veh_id="idm",
    acceleration_controller=(IDMController, {}),
    lane_change_controller=(CentralLaneController, {}),
    routing_controller=(RingBusHumanRouter100SitAware, {}),
    num_vehicles=20)


flow_params = dict(
    # name of the experiment
    exp_tag='ring_cross',

    # name of the flow environment the experiment is running on
    env_name=AccelEnv,

    # name of the network class the experiment is running on
    network=RingCrossNetwork,

    # simulator that is used by the experiment
    simulator='traci',

    # sumo-related parameters (see flow.core.params.SumoParams)
    sim=SumoParams(
        render=True,
        sim_step=0.1,
    ),

    # environment related parameters (see flow.core.params.EnvParams)
    env=EnvParams(
        horizon=2000,
        additional_params=ADDITIONAL_ENV_PARAMS,
    ),

    # network-related parameters (see flow.core.params.NetParams and the
    # network's documentation or ADDITIONAL_NET_PARAMS component)
    net=NetParams(
        additional_params=ADDITIONAL_NET_PARAMS.copy(),
    ),

    # vehicles to be placed in the network at the start of a rollout (see
    # flow.core.params.VehicleParams)
    veh=vehicles,

    # parameters specifying the positioning of vehicles upon initialization/
    # reset (see flow.core.params.InitialConfig)
    # initial=InitialConfig(        
    #     spacing='random',
    #     bunching=0,
    # ), 
    initial=InitialConfig(
        edges_distribution = ['right','bottom'],    
        bunching=20,
    ), 
)
