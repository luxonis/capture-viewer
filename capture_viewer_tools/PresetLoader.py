import tkinter as tk
from tkinter import messagebox
import json
import os

class PresetLoader(tk.Toplevel):
    def __init__(self, parent, preset_folder, load_callback):
        super().__init__(parent)
        self.title("Load Preset")
        self.geometry("300x400")

        self.preset_folder = preset_folder
        self.load_callback = load_callback

        self.preset_listbox = tk.Listbox(self)
        self.preset_listbox.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.load_presets()

        tk.Button(self, text="Load Selected Preset", command=self.load_selected_preset).pack(pady=10)

    def load_presets(self):
        presets = [file for file in os.listdir(self.preset_folder) if file.endswith('.json')]
        for preset in presets:
            self.preset_listbox.insert(tk.END, preset)

    def load_selected_preset(self):
        selection = self.preset_listbox.curselection()
        if not selection:
            messagebox.showerror("Error", "No preset selected.")
            return

        preset_name = self.preset_listbox.get(selection[0])
        preset_path = os.path.join(self.preset_folder, preset_name)

        with open(preset_path, 'r') as file:
            config = json.load(file)

        self.load_callback(config)
        self.destroy()

# Example usage


# Assuming these functions exist elsewhere in your app
# def remove_depthai_from_config(config): pass
# def update_button_values(config, button_values): pass

# How to use:
# on_load_preset(root, "profile_presets", button_values, update_button_values)
