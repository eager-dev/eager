import rospkg
import rosparam
import rosservice
import rostopic
from roslaunch.substitution_args import resolve_args
from six import raise_from


def load_yaml(package_name, object_name):
    try:
        pp = rospkg.RosPack().get_path(package_name)
        filename = pp + "/config/" + object_name + ".yaml"
        params = rosparam.load_file(filename)[0][0]
    except Exception as ex:
        raise_from(RuntimeError(('Unable to load %s from package %s' % (object_name, package_name))), ex)
    return params


def substitute_xml_args(param):
    # substitute string
    if isinstance(param, str):
        param = resolve_args(param)
        return param

    # For every key in the dictionary (not performing deepcopy!)
    if isinstance(param, dict):
        for key in param:
            # If the value is of type `(Ordered)dict`, then recurse with the value
            if isinstance(param[key], dict):
                substitute_xml_args(param[key])
            # Otherwise, add the element to the result
            elif isinstance(param[key], str):
                param[key] = resolve_args(param[key])


def is_namespace_empty(ns):
    # Verifies that there are no topics/services registered to the namespace
    # ROSparam server is not checked.
    srvs_lst = [x for x in rosservice.get_service_list() if ns in x]
    pubs_in, pubs_out = rostopic.get_topic_list()
    topic_lst = []
    for topic, topic_type, node_lst in pubs_in:
        if ns in topic:
            topic_lst.append(topic)
    for topic, topic_type, node_lst in pubs_out:
        if ns in topic:
            topic_lst.append(topic)
    ns_empty = not len(srvs_lst) + len(topic_lst) > 0
    return ns_empty
