from flow.controllers import IDMController, ContinuousRouter
from flow.core.experiment import Experiment
from flow.core.params import SumoParams, EnvParams, \
    InitialConfig, NetParams
from flow.core.params import VehicleParams
from flow.envs import WaveAttenuationPOEnv, AccelEnv, AccelEnvSimple
from flow.networks.ring import RingNetwork, ADDITIONAL_NET_PARAMS
from flow.envs.ring.accel_simple import AccelEnvSimple, ADDITIONAL_ENV_PARAMS

# import sys
# sys.path.append('../')
# sys.path.append('../flow')

sim_params = SumoParams(sim_step=0.1, render=True)

vehicles = VehicleParams()
vehicles.add(veh_id="idm",
             acceleration_controller=(IDMController, {}),
             routing_controller=(ContinuousRouter, {}),
             num_vehicles=5)

env_params=EnvParams(
        horizon=1500,
        additional_params=ADDITIONAL_ENV_PARAMS,
    )

additional_net_params = ADDITIONAL_NET_PARAMS.copy()
net_params = NetParams(additional_params=additional_net_params)

initial_config = InitialConfig(bunching=20)

flow_params = dict(
    exp_tag='ring',
    env_name=AccelEnvSimple,  # using my new environment for the simulation
    network=RingNetwork,
    simulator='traci',
    sim=sim_params,
    env=env_params,
    net=net_params,
    veh=vehicles,
    initial=initial_config,
)

# # number of time steps
# flow_params['env'].horizon = 1500
# exp = Experiment(flow_params)

# # run the sumo simulation
# _ = exp.run(1)