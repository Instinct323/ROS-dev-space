## *********************************************************
##
## File autogenerated for the laser_filters package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [{'name': 'box_frame', 'type': 'str', 'default': 'base_link', 'level': 0, 'description': 'The frame in which the box is created (tf frame_id)', 'min': '', 'max': '', 'srcline': 292, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'std::string', 'cconsttype': 'const char * const'}, {'name': 'min_x', 'type': 'double', 'default': -1.0, 'level': 0, 'description': 'Box starting point in x-axis', 'min': -100.0, 'max': 100.0, 'srcline': 292, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'max_x', 'type': 'double', 'default': 1.0, 'level': 0, 'description': 'Box ending point in x-axis', 'min': -100.0, 'max': 100.0, 'srcline': 292, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'min_y', 'type': 'double', 'default': -1.0, 'level': 0, 'description': 'Box starting point in y-axis', 'min': -100.0, 'max': 100.0, 'srcline': 292, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'max_y', 'type': 'double', 'default': 1.0, 'level': 0, 'description': 'Box ending point in y-axis', 'min': -100.0, 'max': 100.0, 'srcline': 292, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'min_z', 'type': 'double', 'default': -1.0, 'level': 0, 'description': 'Box starting point in z-axis', 'min': -100.0, 'max': 100.0, 'srcline': 292, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'max_z', 'type': 'double', 'default': 1.0, 'level': 0, 'description': 'Box ending point in z-axis', 'min': -100.0, 'max': 100.0, 'srcline': 292, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'invert', 'type': 'bool', 'default': False, 'level': 0, 'description': 'A Boolean to invert the filter', 'min': False, 'max': True, 'srcline': 292, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}], 'groups': [], 'srcline': 247, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

