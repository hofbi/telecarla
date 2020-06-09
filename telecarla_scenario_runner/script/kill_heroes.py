#!/usr/bin/env python3
"""
This module allows killing ego vehicles in a carla simulation.
It can either be integrated in another python moduleby including `kill_heroes.kill_heroes`
Or be called as a standalone script.
"""

import carla
import argparse


def kill_heroes(role_name="hero", host="127.0.0.1", port=2000):
    """Kill all Carla actors specified by role name"""
    client = carla.Client(host, port)
    client.set_timeout(2.0)

    actors = client.get_world().get_actors().filter("vehicle.*")
    killed = 0
    for actor in actors:
        if actor.attributes["role_name"] == role_name:
            print("destroying:", actor.type_id, actor.id, actor.attributes["role_name"])
            killed += 1
            actor.destroy()

    if killed == 0:
        print("No Heroes found!")
    return killed


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Kill all ego/hero vehicles",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", default="127.0.0.1", help="Carla server hostname")
    parser.add_argument("--port", default=2000, type=int, help="Carla server port")
    parser.add_argument("--role-name", default="hero", help="Vehicle role name")
    args = parser.parse_args()

    kill_heroes(args.role_name, args.host, args.port)


if __name__ == "__main__":
    main()
