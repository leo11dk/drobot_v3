from pathlib import Path
import xml.etree.ElementTree as ET


BASE_WORLD_NAME = "basic.sdf"
OUTPUT_WORLD_NAME = "basic_with_center_cylinder.sdf"


def create_center_cylinder_model():
    model = ET.Element("model", name="center_cylinder")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "0 0 0.4 0 0 0"

    link = ET.SubElement(model, "link", name="link")
    ET.SubElement(link, "pose").text = "0 0 0 0 0 0"

    collision = ET.SubElement(link, "collision", name="collision")
    collision_geom = ET.SubElement(collision, "geometry")
    collision_cyl = ET.SubElement(collision_geom, "cylinder")
    ET.SubElement(collision_cyl, "radius").text = "0.35"
    ET.SubElement(collision_cyl, "length").text = "0.8"

    visual = ET.SubElement(link, "visual", name="visual")
    visual_geom = ET.SubElement(visual, "geometry")
    visual_cyl = ET.SubElement(visual_geom, "cylinder")
    ET.SubElement(visual_cyl, "radius").text = "0.35"
    ET.SubElement(visual_cyl, "length").text = "0.8"
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "0.8 0.2 0.2 1"
    ET.SubElement(material, "diffuse").text = "0.8 0.2 0.2 1"

    return model


def main():
    script_dir = Path(__file__).resolve().parent
    worlds_dir = script_dir.parent / "worlds"
    generated_dir = worlds_dir / "generated"
    generated_dir.mkdir(parents=True, exist_ok=True)

    base_world_path = worlds_dir / BASE_WORLD_NAME
    output_world_path = generated_dir / OUTPUT_WORLD_NAME

    if not base_world_path.is_file():
        raise FileNotFoundError(f"Base world not found: {base_world_path}")

    tree = ET.parse(base_world_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("Invalid SDF: <world> element not found")

    existing = world.find("./model[@name='center_cylinder']")
    if existing is not None:
        world.remove(existing)

    world.append(create_center_cylinder_model())

    ET.indent(tree, space="  ")
    tree.write(output_world_path, encoding="utf-8", xml_declaration=True)
    print(f"Generated world: {output_world_path}")


if __name__ == "__main__":
    main()
