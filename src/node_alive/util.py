import roslaunch.config
import roslaunch.xmlloader


def get_node_names(roslaunch_files):
    loader = roslaunch.xmlloader.XmlLoader(resolve_anon=True)
    config = roslaunch.config.load_config_default(roslaunch_files, None, loader=loader, verbose=False,
                                                  assign_machines=False)

    node_names = []
    for node in config.nodes:
        node_names.append(node.namespace + node.name)
    return node_names
