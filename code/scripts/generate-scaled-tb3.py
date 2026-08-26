#!/usr/bin/env python3
"""Generate a uniformly scaled TurtleBot3 SDF and matching URDF."""

from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


SCALE = 3.0
SOURCE_NAME = "turtlebot3_waffle_pi_3d"
TARGET_NAME = "turtlebot3_waffle_pi_3d_large"


def scaled_numbers(text: str, count: int, factor: float) -> str:
    values = text.split()
    values[:count] = [f"{float(value) * factor:.9g}" for value in values[:count]]
    return " ".join(values)


def scale_sdf(source: Path, destination: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise ValueError(f"model element missing: {source}")
    model.set("name", TARGET_NAME)

    for element in root.iter():
        if element.text is None:
            continue
        if element.tag == "pose":
            element.text = scaled_numbers(element.text, 3, SCALE)
        elif element.tag in {"scale", "size"}:
            element.text = scaled_numbers(element.text, 3, SCALE)
        elif element.tag in {"radius", "length", "wheel_separation", "wheel_radius"}:
            element.text = f"{float(element.text) * SCALE:.9g}"
        elif element.tag == "mass":
            element.text = f"{float(element.text) * SCALE ** 3:.9g}"
        elif element.tag in {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"}:
            element.text = f"{float(element.text) * SCALE ** 5:.9g}"

    ET.indent(tree, space="  ")
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def scale_urdf(source: Path, destination: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("name", TARGET_NAME)

    for element in root.iter():
        if element.tag == "origin" and "xyz" in element.attrib:
            element.set("xyz", scaled_numbers(element.get("xyz", ""), 3, SCALE))
        elif element.tag in {"mesh", "box"}:
            attribute = "scale" if element.tag == "mesh" else "size"
            if attribute in element.attrib:
                element.set(
                    attribute,
                    scaled_numbers(element.get(attribute, ""), 3, SCALE),
                )
        elif element.tag == "cylinder":
            for attribute in ("radius", "length"):
                if attribute in element.attrib:
                    element.set(
                        attribute,
                        f"{float(element.get(attribute, '0')) * SCALE:.9g}",
                    )
        elif element.tag == "mass" and "value" in element.attrib:
            element.set("value", f"{float(element.get('value', '0')) * SCALE ** 3:.9g}")
        elif element.tag == "inertia":
            for attribute in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                if attribute in element.attrib:
                    element.set(
                        attribute,
                        f"{float(element.get(attribute, '0')) * SCALE ** 5:.9g}",
                    )

    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def write_model_config(destination: Path) -> None:
    model = ET.Element("model")
    ET.SubElement(model, "name").text = "TurtleBot3(Waffle Pi 3D Large)"
    ET.SubElement(model, "version").text = "1.0"
    ET.SubElement(model, "sdf", version="1.8").text = "model.sdf"
    author = ET.SubElement(model, "author")
    ET.SubElement(author, "name").text = "ROBOTIS and practice repository contributors"
    ET.SubElement(model, "description").text = (
        "Three-times-scale warehouse visualization and collision model"
    )
    tree = ET.ElementTree(model)
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def main() -> None:
    repository = Path(__file__).resolve().parents[2]
    package = repository / "forks/turtlebot3_simulations/turtlebot3_gazebo"
    source_model = package / "models" / SOURCE_NAME
    target_model = package / "models" / TARGET_NAME

    scale_sdf(source_model / "model.sdf", target_model / "model.sdf")
    write_model_config(target_model / "model.config")
    scale_urdf(
        package / "urdf" / f"{SOURCE_NAME}.urdf",
        package / "urdf" / f"{TARGET_NAME}.urdf",
    )
    shutil.copyfile(
        package / "params" / f"{SOURCE_NAME}_bridge.yaml",
        package / "params" / f"{TARGET_NAME}_bridge.yaml",
    )
    generated = ET.parse(target_model / "model.sdf").getroot()
    assert generated.find("model").get("name") == TARGET_NAME
    assert generated.findtext(".//wheel_radius") == "0.099"


if __name__ == "__main__":
    main()
