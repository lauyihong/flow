"""Contains the grid-merge network class."""

from flow.networks.base import Network
from flow.core.params import InitialConfig
from flow.core.params import TrafficLightParams
from numpy import pi, sin, cos
import json

# INFLOW_EDGE_LEN = 100  # length of the inflow edges (needed for resets)
VEHICLE_LENGTH = 5
TURNING_RADIUS = 30
TURNING_RADIUS_NODE = 9
SHIFT_DISTANCE = 0

# ADDITIONAL_NET_PARAMS = {
#     # length of the unit length 
#     "unit_length": 200,
#     # length or the inflow edge
#     "inflow_edge_len": 100,
#     # number of lanes in the main road
#     "main_lanes": 1,
#     # number of lanes in the sub road
#     "sub_lanes": 1,
#     # max speed limit of the network
#     "speed_limit_main": 20,
#     # max speed limit of the sub road
#     "speed_limit_sub": 10,
#     # max speed limit of the extra road
#     "speed_limit_extra": 0.1,
# }

with open('../../tests/yifeng_test/params.json', 'r') as file:
    ADDITIONAL_NET_PARAMS = json.load(file)

class GridMergeNetwork(Network):
    """Network class for highways with a single in-merge.

    This network consists of a single or multi-lane highway network with an
    on-ramp with a variable number of lanes that can be used to generate
    periodic perturbation.

    Requires from net_params:

    * **merge_length** : length of the merge edge
    * **pre_merge_length** : length of the highway leading to the merge
    * **post_merge_length** : length of the highway past the merge
    * **merge_lanes** : number of lanes in the merge
    * **highway_lanes** : number of lanes in the highway
    * **speed_limit** : max speed limit of the network

    Usage
    -----
    >>> from flow.core.params import NetParams
    >>> from flow.core.params import VehicleParams
    >>> from flow.core.params import InitialConfig
    >>> from flow.networks import MergeNetwork
    >>>
    >>> network = MergeNetwork(
    >>>     name='merge',
    >>>     vehicles=VehicleParams(),
    >>>     net_params=NetParams(
    >>>         additional_params={
    >>>             'merge_length': 100,
    >>>             'pre_merge_length': 200,
    >>>             'post_merge_length': 100,
    >>>             'merge_lanes': 1,
    >>>             'highway_lanes': 1,
    >>>             'speed_limit': 30
    >>>         },
    >>>     )
    >>> )
    """

    def __init__(self,
                 name,
                 vehicles,
                 net_params,
                 initial_config=InitialConfig(),
                 traffic_lights=TrafficLightParams()):
        """Initialize a merge network."""
        for p in ADDITIONAL_NET_PARAMS.keys():
            if p not in net_params.additional_params:
                raise KeyError('Network parameter "{}" not supplied'.format(p))

        super().__init__(name, vehicles, net_params, initial_config,
                         traffic_lights)

    def specify_nodes(self, net_params):
        """See parent class."""
        # angle = pi / 4
        # merge = net_params.additional_params["merge_length"]
        # premerge = net_params.additional_params["pre_merge_length"]
        # postmerge = net_params.additional_params["post_merge_length"]

        unitwidth = net_params.additional_params["unit_length"]
        inflow_len = net_params.additional_params["inflow_edge_len"]

        nodes = [
            {
                "id": "inflow_up",
                "x": -inflow_len,
                "y": 0,
                "type": "priority_stop"
            },
            {
                "id": "left_up",
                "y": 0,
                "x": 0,
                "radius": TURNING_RADIUS_NODE,
                "type": "priority_stop"
            },
            {
                "id": "center_up",
                "y": 0,
                "x": unitwidth - SHIFT_DISTANCE,
                "radius": TURNING_RADIUS_NODE,
                "type": "priority_stop"
            },
            {
                "id": "right_up",
                "y": 0,
                "x": unitwidth + unitwidth,
                "radius": TURNING_RADIUS_NODE,
                "type": "priority_stop"
            },
            {
                "id": "inflow_bottom",
                "x": -inflow_len,
                "y": -unitwidth,
                "type": "priority_stop"
            },
            {
                "id": "left_bottom",
                "y": -unitwidth,
                "x": 0,
                "radius": TURNING_RADIUS_NODE,
                "type": "priority_stop"
                # "type": "allway_stop" # priority_stop # priority # traffic_light
            },
            {
                "id": "center_bottom",
                "y": -unitwidth,
                "x": unitwidth + SHIFT_DISTANCE,
                "radius": TURNING_RADIUS_NODE,
                "type": "priority_stop"
            },
            {
                "id": "right_bottom",
                "y": -unitwidth,
                "x": unitwidth + unitwidth,
                "radius": TURNING_RADIUS_NODE,
                "type": "priority_stop"
            },
            {
                "id": "outflow_up",
                "x": unitwidth + unitwidth + inflow_len,
                "y": 0,
                "type": "priority_stop"
            },
            {
                "id": "outflow_bottom",
                "x": unitwidth + unitwidth + inflow_len,
                "y": -unitwidth,
                "type": "priority_stop"
            }
        ]

        return nodes

    def specify_edges(self, net_params):
        """See parent class."""
        # merge = net_params.additional_params["merge_length"]
        # premerge = net_params.additional_params["pre_merge_length"]
        # postmerge = net_params.additional_params["post_merge_length"]

        unitwidth = net_params.additional_params["unit_length"]
        inflow_len = net_params.additional_params["inflow_edge_len"]

        edges = [{
            "id": "inflow_up",
            "type": "inoutType",
            "from": "inflow_up",
            "to": "left_up",
            # "priority": "10",
            "length": inflow_len
        }, {
            "id": "left_up",
            "type": "mainType",
            "from": "left_up",
            "to": "center_up",
            # "priority": "10",
            "length": unitwidth - SHIFT_DISTANCE
        }, {
            "id": "center_up",
            "type": "mainType",
            "from": "center_up",
            "to": "right_up",
            # "priority": "10",
            "length": unitwidth + SHIFT_DISTANCE
        }, {
            "id":"left_vertical",
            "type": "subType",
            "from": "left_up",
            "to": "left_bottom",
            # "priority": "10",
            "length": unitwidth
        }, {
            "id":"center_vertical",
            "type": "extraType",
            "from": "center_up",
            "to": "center_bottom",
            # "priority": "10",
            "length": (unitwidth**2 + SHIFT_DISTANCE**2)**0.5
        }, {
            "id":"right_vertical",
            "type": "subType",
            "from": "right_up",
            "to": "right_bottom",
            # "priority": "10",
            "length": unitwidth
        }, 
        {
            "id": "inflow_bottom",
            "type": "mainType",
            "from": "inflow_bottom",
            "to": "left_bottom",
            # "priority": "10",
            "length": inflow_len
        }, 
        {
            "id": "left_bottom",
            "type": "mainType",
            "from": "left_bottom",
            "to": "center_bottom",
            # "priority": "10",
            "length": unitwidth + SHIFT_DISTANCE
        }, {
            "id": "center_bottom",
            "type": "mainType",
            "from": "center_bottom",
            "to": "right_bottom",
            # "priority": "10",
            "length": unitwidth - SHIFT_DISTANCE
        }, {
            "id": "right_bottom",
            "type": "inoutType",
            "from": "right_bottom",
            "to": "outflow_bottom",
            # "priority": "10",
            "length": inflow_len
        }, {
            "id": "right_up",
            "type": "mainType",
            "from": "right_up",
            "to": "outflow_up",
            # "priority": "10",
            "length": inflow_len
        }]

        return edges

    def specify_types(self, net_params):
        """See parent class."""
        m_lanes = net_params.additional_params["main_lanes"]
        s_lanes = net_params.additional_params["sub_lanes"]
        e_lanes = net_params.additional_params["sub_lanes"]
        m_speed = net_params.additional_params["speed_limit_main"]
        s_speed = net_params.additional_params["speed_limit_sub"]
        e_speed = net_params.additional_params["speed_limit_extra"]
        inout_lane = net_params.additional_params["inout_lanes"]
        types = [{
            "id": "mainType",
            "numLanes": m_lanes,
            "speed": m_speed
        }, {
            "id": "subType",
            "numLanes": s_lanes,
            "speed": s_speed
        }, {
            "id": "extraType",
            "numLanes": e_lanes,
            "speed": e_speed
        },{
            "id": "inoutType",
            "numLanes": inout_lane,
            "speed": m_speed

        }]

        return types

    def specify_routes(self, net_params):
        """See parent class."""
        rts = {
            "inflow_up": ['inflow_up','left_up'],
            "left_up": ["left_up", "center_up", "right_vertical", "right_bottom"],
            "center_up": ["center_up", "right_vertical", "right_bottom"],
            "right_vertical": ["right_vertical", "right_bottom"],
            "left_vertical": ["left_vertical", "left_bottom", "center_bottom", "right_bottom"],
            "center_vertical": ["center_vertical", "center_bottom", "right_bottom"],
            "inflow_bottom": ["inflow_bottom", "left_bottom", "center_bottom", "right_bottom"],
            "left_bottom": ["left_bottom", "center_bottom", "right_bottom"],
            "center_bottom": ["center_bottom", "right_bottom"],
            "right_bottom": ["right_bottom"],
            "right_up": ["right_up"]
        }

        return rts

    '''
    TODO bug happend here, but is this part necessary?
    '''
    # def specify_edge_starts(self):
    #     """See parent class."""
    #     # premerge = self.net_params.additional_params["pre_merge_length"]
    #     # postmerge = self.net_params.additional_params["post_merge_length"]

    #     unitwidth = self.net_params.additional_params["unit_length"]
    #     inflow_len = self.net_params.additional_params["inflow_edge_len"]

    #     edgestarts = [
    #                     ("inflow_up", 0),
    #                     ("left_up", inflow_len + 0.1),
    #                     ("center_up", inflow_len + unitwidth +0.2 + TURNING_RADIUS_NODE),
    #                     ("right_vertical", inflow_len + unitwidth + unitwidth + 0.3 + 2*TURNING_RADIUS),
    #                     ("right_bottom", inflow_len + unitwidth + unitwidth + unitwidth + 0.4),
    #                     ("left_vertical", inflow_len + 0.1 + TURNING_RADIUS),
    #                     ("center_vertical", inflow_len + unitwidth + 0.2 + TURNING_RADIUS),
    #                     ("left_bottom", inflow_len + unitwidth + 0.2 + 2*TURNING_RADIUS),
    #                     ("center_bottom", inflow_len + unitwidth + unitwidth + 0.3 + 2*TURNING_RADIUS),
    #     ]
    #         # ("inflow_highway", 0), 
    #         #           ("left", INFLOW_EDGE_LEN + 0.1),
    #         #           ("center", INFLOW_EDGE_LEN + premerge + 22.6),
    #         #           ("inflow_merge",
    #         #            INFLOW_EDGE_LEN + premerge + postmerge + 22.6),
    #         #           ("bottom",
    #         #            2 * INFLOW_EDGE_LEN + premerge + postmerge + 22.7)

    #     return edgestarts

    # def specify_internal_edge_starts(self):
    #     """See parent class."""
    #     # premerge = self.net_params.additional_params["pre_merge_length"]
    #     # postmerge = self.net_params.additional_params["post_merge_length"]

    #     unitwidth = self.net_params.additional_params["unit_length"]
    #     inflow_len = self.net_params.additional_params["inflow_edge_len"]

    #     internal_edgestarts = [
    #         # ("inflow_up", 0),
    #         (":left_up_0", inflow_len),
    #         (":center_up_0", inflow_len + unitwidth +0.1),
    #         (":right_vertical_0", inflow_len + unitwidth + unitwidth + 0.2),
    #         (":right_bottom_0", inflow_len + unitwidth + unitwidth + unitwidth + 0.3 + TURNING_RADIUS),
    #         (":left_vertical_0", inflow_len),
    #         (":center_vertical_0", inflow_len + unitwidth + 0.1),
    #         (":left_bottom_0", inflow_len + unitwidth + 0.1 + TURNING_RADIUS),
    #         (":center_bottom_0", inflow_len + unitwidth + unitwidth + 0.2 + TURNING_RADIUS),
    #     ]
    #         # (":left", INFLOW_EDGE_LEN), (":center",
    #         #                              INFLOW_EDGE_LEN + premerge + 0.1),
    #         # (":bottom", 2 * INFLOW_EDGE_LEN + premerge + postmerge + 22.6)
        

    #     return internal_edgestarts
