import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path
import shlex
import subprocess
import sys
import math
import xml.etree.ElementTree as ET

root = tk.Tk()
root.title("World Generator UI")
root.geometry("1200x500")

root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=2)
root.grid_columnconfigure(1, weight=0)
root.grid_columnconfigure(2, weight=2)
root.grid_columnconfigure(3, weight=0)
root.grid_columnconfigure(4, weight=2)
root.grid_columnconfigure(5, weight=0)
root.grid_columnconfigure(6, weight=2)

left = ttk.Frame(root, padding=10)
left.grid(row=0, column=0, sticky="nsew")

sep = ttk.Separator(root, orient="vertical")
sep.grid(row=0, column=1, sticky="ns")

middle = ttk.Frame(root, padding=10)
middle.grid(row=0, column=2, sticky="nsew")

sep2 = ttk.Separator(root, orient="vertical")
sep2.grid(row=0, column=3, sticky="ns")

world_col = ttk.Frame(root, padding=10)
world_col.grid(row=0, column=4, sticky="nsew")

sep3 = ttk.Separator(root, orient="vertical")
sep3.grid(row=0, column=5, sticky="ns")

right = ttk.Frame(root, padding=10)
right.grid(row=0, column=6, sticky="nsew")

# Headers
ttk.Label(left, text="Select World").pack(anchor="w")
ttk.Label(middle, text="Create World").pack(anchor="w")
ttk.Label(world_col, text="Select .world").pack(anchor="w")
ttk.Label(right, text="Selection").pack(anchor="w")

generated_world_dir = Path(__file__).resolve().parent.parent / "worlds" / "generated"
worlds_dir = Path(__file__).resolve().parent.parent / "worlds"
world_generator_script = Path(__file__).resolve().parent / "world_generator.py"
workspace_dir = Path(__file__).resolve().parents[3]
saved_worlds = []
saved_dot_worlds = []
create_worlds = ["Stationary", "Moving", "Random"]
saved_world_paths = {}

selected_world = tk.StringVar(value="(none)")

ttk.Label(right, text="Selected:").pack(anchor="w", pady=(10, 0))
ttk.Label(right, textvariable=selected_world).pack(anchor="w", pady=(0, 10))

# Create ONE action button (disabled by default)
action_btn = ttk.Button(right, text="Select something first", state="disabled")
action_btn.pack(pady=5, fill="x")

left_buttons = ttk.Frame(left)
left_buttons.pack(fill="both", expand=True)

middle_buttons = ttk.Frame(middle)
middle_buttons.pack(fill="both", expand=True)

world_buttons = ttk.Frame(world_col)
world_buttons.pack(fill="both", expand=True)


def load_saved_worlds():
    return sorted(
        world_file.name for world_file in generated_world_dir.glob("*.sdf")
        if world_file.is_file()
    )


def load_saved_dot_worlds():
    return sorted(
        world_file.relative_to(worlds_dir).as_posix()
        for world_file in worlds_dir.rglob("*.world")
        if world_file.is_file()
    )


def parse_pose_text(pose_text):
    if not pose_text:
        return 0.0, 0.0, 0.0
    values = [float(v) for v in pose_text.split()]
    while len(values) < 6:
        values.append(0.0)
    return values[0], values[1], values[5]


def compose_pose(parent_pose, child_pose):
    px, py, pyaw = parent_pose
    cx, cy, cyaw = child_pose
    x = px + math.cos(pyaw) * cx - math.sin(pyaw) * cy
    y = py + math.sin(pyaw) * cx + math.cos(pyaw) * cy
    return x, y, pyaw + cyaw


def point_in_obstacle(px, py, obstacle, clearance):
    ox = px - obstacle["x"]
    oy = py - obstacle["y"]
    yaw = obstacle["yaw"]
    local_x = math.cos(-yaw) * ox - math.sin(-yaw) * oy
    local_y = math.sin(-yaw) * ox + math.cos(-yaw) * oy

    if obstacle["type"] == "cylinder":
        return (local_x * local_x + local_y * local_y) <= (obstacle["radius"] + clearance) ** 2
    if obstacle["type"] == "box":
        return (
            abs(local_x) <= (obstacle["sx"] / 2.0 + clearance)
            and abs(local_y) <= (obstacle["sy"] / 2.0 + clearance)
        )
    return False


def extract_obstacles(world_path):
    obstacles = []
    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        return obstacles

    for model in world.findall("model"):
        model_pose = parse_pose_text(model.findtext("pose"))
        for link in model.findall("link"):
            link_pose = compose_pose(model_pose, parse_pose_text(link.findtext("pose")))
            for collision in link.findall("collision"):
                collision_pose = compose_pose(link_pose, parse_pose_text(collision.findtext("pose")))
                geometry = collision.find("geometry")
                if geometry is None:
                    continue

                box = geometry.find("box")
                if box is not None:
                    size_text = box.findtext("size")
                    if size_text:
                        sx, sy, _ = [float(v) for v in size_text.split()]
                        obstacles.append(
                            {
                                "type": "box",
                                "x": collision_pose[0],
                                "y": collision_pose[1],
                                "yaw": collision_pose[2],
                                "sx": sx,
                                "sy": sy,
                            }
                        )
                    continue

                cylinder = geometry.find("cylinder")
                if cylinder is not None:
                    radius_text = cylinder.findtext("radius")
                    if radius_text:
                        obstacles.append(
                            {
                                "type": "cylinder",
                                "x": collision_pose[0],
                                "y": collision_pose[1],
                                "yaw": collision_pose[2],
                                "radius": float(radius_text),
                            }
                        )
    return obstacles


def find_safe_spawn(world_path):
    try:
        obstacles = extract_obstacles(world_path)
    except Exception:
        return 0.0, 0.0, 0.05

    clearance = 0.45
    candidates = [(0.0, 0.0)]
    for radius in [0.8, 1.4, 2.0, 2.6, 3.2, 4.0, 5.0]:
        candidates.extend(
            [
                (radius, 0.0), (-radius, 0.0), (0.0, radius), (0.0, -radius),
                (radius, radius), (radius, -radius), (-radius, radius), (-radius, -radius),
            ]
        )

    for cx, cy in candidates:
        if not any(point_in_obstacle(cx, cy, obstacle, clearance) for obstacle in obstacles):
            return cx, cy, 0.05
    return 0.0, 0.0, 0.05


def refresh_saved_world_buttons():
    global saved_worlds, saved_world_paths
    saved_worlds = load_saved_worlds()
    saved_world_paths = {}
    for child in left_buttons.winfo_children():
        child.destroy()
    for world_name in saved_worlds:
        saved_world_paths[world_name] = generated_world_dir / world_name
        ttk.Button(
            left_buttons,
            text=world_name,
            command=lambda name=world_name: on_select_world(name)
        ).pack(pady=5, fill="x")


def refresh_saved_dot_world_buttons():
    global saved_dot_worlds, saved_world_paths
    saved_dot_worlds = load_saved_dot_worlds()
    for child in world_buttons.winfo_children():
        child.destroy()
    for world_name in saved_dot_worlds:
        saved_world_paths[world_name] = worlds_dir / world_name
        ttk.Button(
            world_buttons,
            text=world_name,
            command=lambda name=world_name: on_select_world(name)
        ).pack(pady=5, fill="x")


def action_world():
    world = selected_world.get()
    if world in saved_world_paths:
        world_path = saved_world_paths[world]
        if not world_path.is_file():
            messagebox.showerror("Load World", f"World file not found:\n{world_path}")
            return

        spawn_x, spawn_y, spawn_z = find_safe_spawn(world_path)
        cmd = (
            f"cd {shlex.quote(str(workspace_dir))} && "
            "colcon build --packages-select drobot_description drobot_simulation && "
            "source install/setup.bash && "
            f"ros2 launch drobot_simulation sim.launch.py "
            f"world:={shlex.quote(str(world_path))} "
            f"spawn_x:={spawn_x} spawn_y:={spawn_y} spawn_z:={spawn_z}"
        )
        subprocess.Popen(["bash", "-lc", cmd])
        print(f"Launching world: {world_path}")
        messagebox.showinfo(
            "Load World",
            f"Launching '{world}'.\nRunning build + source + launch in a new shell."
        )
    elif world in create_worlds:
        if not world_generator_script.is_file():
            messagebox.showerror("Create World", f"Script not found:\n{world_generator_script}")
            return

        try:
            subprocess.run(
                [sys.executable, str(world_generator_script)],
                cwd=str(world_generator_script.parent),
                check=True,
            )
        except subprocess.CalledProcessError as exc:
            messagebox.showerror("Create World", f"Generation failed:\n{exc}")
            return

        refresh_saved_world_buttons()
        refresh_saved_dot_world_buttons()
        print(f"Running world generator for: {world}")
        messagebox.showinfo("Create World", f"Generated world for '{world}' and refreshed list.")


def on_select_world(world_name):
    selected_world.set(world_name)
    print(f"Selected world: {world_name}")

    # Update the ONE button instead of creating new ones
    if world_name in saved_world_paths:
        action_btn.config(text="Load World", state="normal", command=action_world)
    elif world_name in create_worlds:
        action_btn.config(text="Create World", state="normal", command=action_world)
    else:
        action_btn.config(text="Unknown selection", state="disabled")


def button_creation():
    refresh_saved_world_buttons()
    refresh_saved_dot_world_buttons()
    for w in create_worlds:
        ttk.Button(middle_buttons, text=w, command=lambda name=w: on_select_world(name)).pack(pady=5, fill="x")

button_creation()
root.mainloop()
