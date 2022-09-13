# node_alive

[![CI](https://github.com/tue-robotics/node_alive/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/node_alive/actions/workflows/main.yml)

`node_alive_server` checks if all nodes in the `node_list` are still alive. Nodes can be added to the node_list with use of `node_alive_add [launch_file]`. `node_alive_add` scans recursively through all nodes that are being launched with use of the specified launch file and adds these to the node_list.
