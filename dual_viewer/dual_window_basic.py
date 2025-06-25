# Basic Tkinter app with two vertically stacked file loading and viewing sections for .npy/.png files
# Shows metadata only in folder selection menu, hides it during image viewing

import os
import json
import tkinter as tk
from tkinter import ttk, Canvas, Scrollbar
import numpy as np
from PIL import Image, ImageTk

from FileSelection import FolderExplorer

class FileSection(tk.LabelFrame):
    def __init__(self, master):
        super().__init__(master, bd=2, relief="groove", bg="#1c1c1e", fg="white")
        self.folder_path = tk.StringVar()
        self.file_types = []
        self.selected_type = tk.StringVar()
        self.files = []
        self.current_index = 0
        self.original_image = None
        self.scaled_image = None
        self.image_size = (0, 0)
        self.width_var = tk.StringVar()
        self.height_var = tk.StringVar()

        self.image_label = None
        self.file_label = None
        self.width_entry = None
        self.height_entry = None

        self.metadata_canvas = Canvas(self, width=300, height=150, bg="#1c1c1e", highlightthickness=0)
        self.metadata_scrollbar = Scrollbar(self, orient="vertical", command=self.metadata_canvas.yview)
        self.metadata_frame = tk.Frame(self.metadata_canvas, bg="#1c1c1e")
        self.metadata_canvas.create_window((0, 0), window=self.metadata_frame, anchor="nw")
        self.metadata_canvas.configure(yscrollcommand=self.metadata_scrollbar.set)

        self.build_menu()

    def build_menu(self):
        for widget in self.winfo_children():
            if widget not in [self.metadata_canvas, self.metadata_scrollbar]:
                widget.destroy()

        tk.Label(self, textvariable=self.folder_path, bg="#1c1c1e", fg="white", font=("Arial", 12, "bold"))\
            .pack(pady=(10, 0))
        tk.Entry(self, textvariable=self.folder_path, width=50, bg="#2c2c2e", fg="white", insertbackground="white").pack(pady=5)
        tk.Button(self, text="Browse", command=self.browse_folder, bg="#5e5ce6", fg="white", relief="flat").pack()
        self.type_combo = ttk.Combobox(self, textvariable=self.selected_type)
        self.type_combo.pack(pady=5)
        tk.Button(self, text="Load", command=self.load_files, bg="#5e5ce6", fg="white", relief="flat").pack(pady=5)

        self.metadata_canvas.pack(side="left", fill="both", expand=True, padx=10)
        self.metadata_scrollbar.pack(side="right", fill="y")
        self.metadata_frame.bind("<Configure>", lambda e: self.metadata_canvas.configure(scrollregion=self.metadata_canvas.bbox("all")))

        if self.folder_path.get():
            self.analyze_folder()

    def browse_folder(self):
        def set_path(path):
            self.folder_path.set(path)
            self.analyze_folder()

        FolderExplorer(self, on_select_callback=set_path, initial_dir=self.folder_path.get() or ".")

    def analyze_folder(self):
        path = self.folder_path.get()
        if not os.path.isdir(path):
            return
        files = os.listdir(path)
        type_set = set()
        for f in files:
            if '_' in f:
                t = f.split('_')[0]
                ext = os.path.splitext(f)[1][1:]
                if ext in ['npy', 'png']:
                    type_set.add(f"{t} ({ext})")

        sorted_types = sorted(type_set)
        self.file_types = sorted_types
        self.type_combo['values'] = sorted_types
        if 'depth (npy)' in sorted_types:
            self.selected_type.set('depth (npy)')
        elif sorted_types:
            self.selected_type.set(sorted_types[0])

        self.load_metadata(path)

    def load_metadata(self, path):
        for widget in self.metadata_frame.winfo_children():
            widget.destroy()

        metadata_path = os.path.join(path, "metadata.json")
        if os.path.isfile(metadata_path):
            with open(metadata_path, 'r') as f:
                try:
                    data = json.load(f)
                    for k, v in data.items():
                        tk.Label(self.metadata_frame, text=f"{k}: {v}", anchor='w', justify='left', bg="#1c1c1e", fg="white").pack(fill='x')
                except json.JSONDecodeError:
                    tk.Label(self.metadata_frame, text="Invalid metadata.json", fg="red", bg="#1c1c1e").pack()
        else:
            tk.Label(self.metadata_frame, text="No metadata.json present", fg="gray", bg="#1c1c1e").pack()

    def load_files(self):
        type_str = self.selected_type.get()
        if not type_str:
            return
        base_type, ext = type_str.split(' (')
        ext = ext.rstrip(')')
        self.files = [f for f in os.listdir(self.folder_path.get())
                      if f.startswith(base_type + '_') and f.endswith(ext)]
        self.files.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))
        self.current_index = 0
        self.build_viewer()

    def build_viewer(self):
        for widget in self.winfo_children():
            if widget not in [self.metadata_canvas, self.metadata_scrollbar]:
                widget.destroy()

        self.metadata_canvas.pack_forget()
        self.metadata_scrollbar.pack_forget()

        tk.Label(self, textvariable=self.folder_path, bg="#1c1c1e", fg="white", font=("Arial", 12, "bold")).pack(pady=(10, 0))

        nav_frame = tk.Frame(self, bg="#1c1c1e")
        nav_frame.pack(pady=5)

        tk.Button(nav_frame, text="<", command=self.prev_file, bg="#5e5ce6", fg="white", relief="flat").pack(side="left")
        self.file_label = tk.Label(nav_frame, bg="#1c1c1e", fg="white")
        self.file_label.pack(side="left", padx=10)
        tk.Button(nav_frame, text=">", command=self.next_file, bg="#5e5ce6", fg="white", relief="flat").pack(side="left")

        tk.Button(self, text="MENU", command=self.return_to_menu, bg="#5e5ce6", fg="white", relief="flat").pack(pady=5)

        resize_frame = tk.Frame(self, bg="#1c1c1e")
        resize_frame.pack(pady=5)
        tk.Label(resize_frame, text="Size:", bg="#1c1c1e", fg="white").pack(side="left")

        self.width_entry = tk.Entry(resize_frame, textvariable=self.width_var, width=6)
        self.height_entry = tk.Entry(resize_frame, textvariable=self.height_var, width=6)
        self.width_entry.pack(side="left")
        tk.Label(resize_frame, text="x", bg="#1c1c1e", fg="white").pack(side="left")
        self.height_entry.pack(side="left", padx=(0, 10))
        self.max_label = tk.Label(resize_frame, bg="#1c1c1e", fg="gray")
        self.max_label.pack(side="left")

        self.width_entry.bind("<Return>", self.update_resize)
        self.height_entry.bind("<Return>", self.update_resize)

        self.image_label = tk.Label(self, bg="#1c1c1e")
        self.image_label.pack()

        self.update_image_view()

    def update_image_view(self):
        file_path = os.path.join(self.folder_path.get(), self.files[self.current_index])
        self.file_label.config(text=self.files[self.current_index])

        if file_path.endswith(".npy"):
            arr = np.load(file_path)
            self.original_image = Image.fromarray((arr / arr.max() * 255).astype(np.uint8))
        elif file_path.endswith(".png"):
            self.original_image = Image.open(file_path)
        else:
            return

        self.image_size = self.original_image.size
        if not self.width_var.get() or not self.height_var.get():
            self.width_var.set(str(self.image_size[0]))
            self.height_var.set(str(self.image_size[1]))

        self.max_label.config(text=f"(max: {self.image_size[0]}x{self.image_size[1]})")
        self.display_rescaled_image()

    def update_resize(self, event=None):
        try:
            if event and event.widget == self.width_entry:
                new_width = int(self.width_var.get())
                aspect_ratio = self.image_size[1] / self.image_size[0]
                new_height = int(new_width * aspect_ratio)
                self.height_var.set(str(new_height))
            elif event and event.widget == self.height_entry:
                new_height = int(self.height_var.get())
                aspect_ratio = self.image_size[0] / self.image_size[1]
                new_width = int(new_height * aspect_ratio)
                self.width_var.set(str(new_width))
            self.display_rescaled_image()
        except ValueError:
            pass

    def display_rescaled_image(self):
        if self.original_image:
            try:
                width = int(self.width_var.get())
                height = int(self.height_var.get())
                self.scaled_image = ImageTk.PhotoImage(self.original_image.resize((width, height), Image.LANCZOS))
                self.image_label.config(image=self.scaled_image)
                self.image_label.image = self.scaled_image
            except Exception:
                pass

    def return_to_menu(self):
        self.build_menu()

    def prev_file(self):
        if self.current_index > 0:
            self.current_index -= 1
            self.update_image_view()

    def next_file(self):
        if self.current_index < len(self.files) - 1:
            self.current_index += 1
            self.update_image_view()

class DualApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dual File Viewer")
        self.geometry("1600x1600")
        self.configure(bg="#1c1c1e")

        top_section = FileSection(self)
        bottom_section = FileSection(self)

        top_section.pack(side="top", fill="both", expand=True, padx=20, pady=20)
        bottom_section.pack(side="top", fill="both", expand=True, padx=20, pady=20)

if __name__ == '__main__':
    app = DualApp()
    style = ttk.Style()
    style.theme_use('default')
    style.configure("TCombobox", fieldbackground="#2c2c2e", background="#2c2c2e", foreground="white")
    app.mainloop()
