#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import yaml
import rocon_python_utils


INVALID_PARAM = ['name', 'description', 'uuid']

##############################################################################
# Methods
##############################################################################


def load_parameters_from_key_value_msg(parameter_key_value_msg, namespace, name, load):
    for kv in parameter_key_value_msg:
        load_parameter(kv.key, kv.value, namespace, name, load)

def load_parameters_from_resource(parameter_resource_name, namespace, name, load):
    filepath = rocon_python_utils.ros.find_resource_from_string(parameter_resource_name, extension='parameters')
    load_parameters_from_file(filepath, namespace, name, load)

def load_parameters_from_file(parameter_file_path, namespace, name, load):
    filepath = parameter_file_path
    with open(filepath) as f:
        params = yaml.load(f)
        for k, v in params.items():
            load_parameter(k, v, namespace, name, load)

def load_parameter(key, value, namespace, name, load):
    if key in INVALID_PARAM:
        if load:
            rospy.logwarn("Service Manager : %s%s [%s]" % (str(k), ' is prohibitted parameter. Ignoring...', name))
    else:
        param_name = namespace + '/' + key
        if load:
            try:
                rospy.set_param(param_name, eval(value))
            except (NameError, TypeError, SyntaxError):
                rospy.set_param(param_name, value)
        else:
            rospy.delete_param(param_name)
