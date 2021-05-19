import rospkg, rosparam
from roslaunch.substitution_args import resolve_args
from future.utils import raise_from


def load_yaml(package_name, object_name):
    try:
        pp = rospkg.RosPack().get_path(package_name)
        filename = pp + "/config/" + object_name + ".yaml"
        params = rosparam.load_file(filename)[0][0]
    except Exception as ex:
        raise_from(RuntimeError(('Unable to load %s from package %s' % object_name, package_name)), ex)
    return params


def substitute_xml_args(dictionary):
    # For every key in the dictionary
    for key in dictionary:
        # If the value is of type `dict`, then recurse with the value
        if isinstance(dictionary[key], dict):
            substitute_xml_args(dictionary[key])
        # Otherwise, add the element to the result
        elif isinstance(dictionary[key], str):
            dictionary[key] = resolve_args(dictionary[key])