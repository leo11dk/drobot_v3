import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path
import shlex
import subprocess

root = tk.Tk()
root.title("World Generator UI")
root.geometry("900x500")

root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=2)
root.grid_columnconfigure(1, weight=0)
root.grid_columnconfigure(2, weight=2)
root.grid_columnconfigure(3, weight=0)
root.grid_columnconfigure(4, weight=2)

left = ttk.Frame(root, padding=10)
left.grid(row=0, column=0, sticky="nsew")

sep = ttk.Separator(root, orient="vertical")
sep.grid(row=0, column=1, sticky="ns")

middle = ttk.Frame(root, padding=10)
middle.grid(row=0, column=2, sticky="nsew")

sep2 = ttk.Separator(root, orient="vertical")
sep2.grid(row=0, column=3, sticky="ns")

right = ttk.Frame(root, padding=10)
right.grid(row=0, column=4, sticky="nsew")

# Headers
ttk.Label(left, text="Select World").pack(anchor="w")
ttk.Label(middle, text="Create World").pack(anchor="w")
ttk.Label(right, text="Selection").pack(anchor="w")

generated_world_dir = Path(__file__).resolve().parent.parent / "worlds" / "generated"
workspace_dir = Path(__file__).resolve().parents[3]
saved_worlds = sorted(
    world_file.name for world_file in generated_world_dir.glob("*.sdf")
    if world_file.is_file()
)
create_worlds = ["Stationary", "Moving", "Random"]

selected_world = tk.StringVar(value="(none)")

ttk.Label(right, text="Selected:").pack(anchor="w", pady=(10, 0))
ttk.Label(right, textvariable=selected_world).pack(anchor="w", pady=(0, 10))

# Create ONE action button (disabled by default)
action_btn = ttk.Button(right, text="Select something first", state="disabled")
action_btn.pack(pady=5, fill="x")


def action_world():
    world = selected_world.get()
    if world in saved_worlds:
        world_path = generated_world_dir / world
        if not world_path.is_file():
            messagebox.showerror("Load World", f"World file not found:\n{world_path}")
            return

        cmd = (
            f"cd {shlex.quote(str(workspace_dir))} && "
            "colcon build --packages-select drobot_description drobot_simulation && "
            "source install/setup.bash && "
            f"ros2 launch drobot_simulation sim.launch.py world:={shlex.quote(str(world_path))}"
        )
        subprocess.Popen(["bash", "-lc", cmd])
        print(f"Launching world: {world_path}")
        messagebox.showinfo(
            "Load World",
            f"Launching '{world}'.\nRunning build + source + launch in a new shell."
        )
    elif world in create_worlds:
        print(f"Creating world: {world}")
        messagebox.showinfo("Create World", f"World '{world}' created successfully.")


def on_select_world(world_name):
    selected_world.set(world_name)
    print(f"Selected world: {world_name}")

    # Update the ONE button instead of creating new ones
    if world_name in saved_worlds:
        action_btn.config(text="Load World", state="normal", command=action_world)
    elif world_name in create_worlds:
        action_btn.config(text="Create World", state="normal", command=action_world)
    else:
        action_btn.config(text="Unknown selection", state="disabled")


def button_creation():
    for w in saved_worlds:
        ttk.Button(left, text=w, command=lambda name=w: on_select_world(name)).pack(pady=5, fill="x")
    for w in create_worlds:
        ttk.Button(middle, text=w, command=lambda name=w: on_select_world(name)).pack(pady=5, fill="x")

button_creation()
root.mainloop()
