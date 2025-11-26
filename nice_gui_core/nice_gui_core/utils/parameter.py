from rclpy.node import Node
from rclpy.parameter import Parameter, ParameterValue


def get_parameter(node: Node, parameter: str, parameter_type: Parameter) -> ParameterValue:
    '''
    Get the parameter value from the node.

    Parameters
    ------------
    node: Node
        ROS2 node
    parameter: str
        parameter name
    parameter_type: Parameter
        parameter type

    Returns
    ----------
    ParameterValue
        parameter value
    '''
    if not node.has_parameter(parameter):
        node.declare_parameter(parameter, parameter_type)
    return node.get_parameter(parameter).get_parameter_value()
