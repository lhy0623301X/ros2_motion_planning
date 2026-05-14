#!/usr/bin/env python3

import argparse
import re
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


DEFAULT_SKIP = {"ground_plane"}
OLD_MODEL_PREFIX = "model://sim_env/models/"
NEW_MODEL_PREFIX = "model://"
MODEL_URI_RE = re.compile(r"model://(?:sim_env/models/)?([^/]+)/")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Spawn models from a world file into an already running empty Gazebo world."
    )
    parser.add_argument(
        "--world-file",
        required=True,
        help="Absolute path to the source .world file.",
    )
    parser.add_argument(
        "--service-name",
        default="/spawn_entity",
        help="Gazebo spawn service name.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.5,
        help="Delay in seconds between model spawns.",
    )
    parser.add_argument(
        "--skip-model",
        action="append",
        default=[],
        help="Model name to skip. Can be provided multiple times.",
    )
    return parser.parse_args()


def wrap_model_xml(model_xml: str, sdf_version: str) -> str:
    return f"<sdf version='{sdf_version}'>{model_xml}</sdf>"


def normalize_model_xml(model_xml: str) -> str:
    return model_xml.replace(OLD_MODEL_PREFIX, NEW_MODEL_PREFIX)


def maybe_convert_model_to_include(name: str, model_xml: str) -> str:
    try:
        model = ET.fromstring(model_xml)
    except ET.ParseError:
        return model_xml

    if model.tag != "model":
        return model_xml

    uris = [uri.text for uri in model.findall(".//uri") if uri.text]
    base_model = None
    for uri in uris:
        match = MODEL_URI_RE.match(uri)
        if match:
            base_model = match.group(1)
            break

    # Keep primitive-only models such as walls as raw model XML.
    if base_model is None:
        return model_xml

    include = ET.Element("include")
    uri_elem = ET.SubElement(include, "uri")
    uri_elem.text = f"model://{base_model}"

    name_elem = ET.SubElement(include, "name")
    name_elem.text = name

    pose_text = model.findtext("pose", default="")
    if pose_text:
        pose_elem = ET.SubElement(include, "pose")
        pose_elem.text = pose_text

    static_text = model.findtext("static", default="")
    if static_text:
        static_elem = ET.SubElement(include, "static")
        static_elem.text = static_text

    return ET.tostring(include, encoding="unicode")


def load_models(world_file: str):
    tree = ET.parse(world_file)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"No <world> tag found in {world_file}")
    sdf_version = root.attrib.get("version", "1.7")
    spawnables = []
    for child in world:
        if child.tag == "model":
            name = child.attrib.get("name", "")
            if not name:
                continue
            spawnables.append((name, ET.tostring(child, encoding="unicode")))
        elif child.tag == "include":
            uri = child.findtext("uri", default="")
            if not uri:
                continue
            name = child.findtext("name", default=uri.rstrip("/").split("/")[-1])
            spawnables.append((name, ET.tostring(child, encoding="unicode")))
    return sdf_version, spawnables


def main():
    args = parse_args()
    skip = set(DEFAULT_SKIP)
    skip.update(args.skip_model)

    sdf_version, spawnables = load_models(args.world_file)

    rclpy.init(args=None)
    node = rclpy.create_node("world_model_loader")
    client = node.create_client(SpawnEntity, args.service_name)

    node.get_logger().info(f"Waiting for spawn service: {args.service_name}")
    if not client.wait_for_service(timeout_sec=30.0):
        node.get_logger().error(f"Service not available: {args.service_name}")
        rclpy.shutdown()
        return 1

    spawned = 0
    failed = 0

    for name, raw_xml in spawnables:
        if name in skip:
            node.get_logger().info(f"Skipping model: {name}")
            continue

        model_xml = normalize_model_xml(raw_xml)
        model_xml = maybe_convert_model_to_include(name, model_xml)
        request = SpawnEntity.Request()
        request.name = name
        request.xml = wrap_model_xml(model_xml, sdf_version)
        request.robot_namespace = ""
        request.initial_pose = Pose()
        request.reference_frame = "world"

        node.get_logger().info(f"Spawning model: {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)

        if not future.done():
            failed += 1
            node.get_logger().error(f"Timed out spawning model: {name}")
            continue

        response = future.result()
        if response is None:
            failed += 1
            node.get_logger().error(f"Service call failed for model: {name}")
            continue

        if response.success:
            spawned += 1
            node.get_logger().info(f"Spawned model: {name}")
        else:
            failed += 1
            node.get_logger().error(
                f"Failed to spawn model {name}: {response.status_message}"
            )

        if args.delay > 0.0:
            time.sleep(args.delay)

    node.get_logger().info(
        f"Finished loading models from {args.world_file}. "
        f"Spawned: {spawned}, Failed: {failed}"
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if failed == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
