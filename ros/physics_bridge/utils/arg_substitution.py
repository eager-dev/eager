from roslaunch.substitution_args import resolve_args

def substitute_xml_args(dictionary):
    # For every key in the dictionary
    for key in dictionary:
        # If the value is of type `dict`, then recurse with the value
        if isinstance(dictionary[key], dict):
            substitute_xml_args(dictionary[key])
        # Otherwise, add the element to the result
        elif isinstance(dictionary[key], str):
            dictionary[key] = resolve_args(dictionary[key])