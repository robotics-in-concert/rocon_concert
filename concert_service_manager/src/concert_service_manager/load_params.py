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


def load_parameters_from_file(parameter_file, namespace, name, load):
    filepath = rocon_python_utils.ros.find_resource_from_string(parameter_file, extension='parameters')

    with open(filepath) as f:
        params = yaml.load(f)
        for k, v in params.items():
            if k in INVALID_PARAM:
                if load:
                    rospy.logwarn("Service Manager : %s%s [%s]" % (str(k), ' is prohibitted parameter. Ignoring...', name))
                continue
            param_name = namespace + '/' + k
            if load:
                rospy.set_param(param_name, v)
            else:
                rospy.delete_param(param_name)


def load_parameters_from_cache(catche_file_path, namespace, name, load):
    filepath = catche_file_path

    with open(filepath) as f:
        params = yaml.load(f)
        for k, v in params.items():
            if k in INVALID_PARAM:
                if load:
                    rospy.logwarn("Service Manager : %s%s [%s]" % (str(k), ' is prohibitted parameter. Ignoring...', name))
                continue
            param_name = namespace + '/' + k
            if load:
                rospy.set_param(param_name, v)
            else:
                rospy.delete_param(param_name)
