# Basic Tkinter app with two vertically stacked file loading and viewing sections for .npy/.png files and metadata display

import os
import json
import tkinter as tk
from tkinter import filedialog, ttk, Canvas, Scrollbar
import numpy as np
from PIL import Image, ImageTk

class FileSection(tk.LabelFrame):
    def __init__(self, master):
        super().__init__(master, bd=2, relief="groove", bg="#1c1c1e", fg="white")
        self.folder_path = tk.StringVar()
        self.file_types = []
        self.selected_type = tk.StringVar()
        self.files = []
        self.current_index = 0

        self.build_menu()

    def build_menu(self):
        for widget in self.winfo_children():
            widget.destroy()

        tk.Label(self, textvariable=self.folder_path, bg="#1c1c1e", fg="white", font=("Arial", 12, "bold"))\
            .pack(pady=(10, 0))
        tk.Entry(self, textvariable=self.folder_path, width=50, bg="#2c2c2e", fg="white", insertbackground="white").pack(pady=5)
        tk.Button(self, text="Browse", command=self.browse_folder, bg="#5e5ce6", fg="white", relief="flat").pack()
        self.type_combo = ttk.Combobox(self, textvariable=self.selected_type)
        self.type_combo.pack(pady=5)
        tk.Button(self, text="Load", command=self.load_files, bg="#5e5ce6", fg="white", relief="flat").pack(pady=5)

        self.metadata_canvas = Canvas(self, width=300, height=150, bg="#1c1c1e", highlightthickness=0)
        self.metadata_scrollbar = Scrollbar(self, orient="vertical", command=self.metadata_canvas.yview)
        self.metadata_frame = tk.Frame(self.metadata_canvas, bg="#1c1c1e")

        self.metadata_canvas.create_window((0, 0), window=self.metadata_frame, anchor="nw")
        self.metadata_canvas.configure(yscrollcommand=self.metadata_scrollbar.set)

        self.metadata_canvas.pack(side="left", fill="both", expand=True, padx=10)
        self.metadata_scrollbar.pack(side="right", fill="y")

        self.metadata_frame.bind("<Configure>", lambda e: self.metadata_canvas.configure(scrollregion=self.metadata_canvas.bbox("all")))

        if self.folder_path.get():
            self.analyze_folder()

    def browse_folder(self):
        path = filedialog.askdirectory()
        if path:
            self.folder_path.set(path)
            self.analyze_folder()

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
        self.display_file()

    def display_file(self):
        for widget in self.winfo_children():
            widget.destroy()

        tk.Label(self, textvariable=self.folder_path, bg="#1c1c1e", fg="white", font=("Arial", 12, "bold"))\
            .pack(pady=(10, 0))

        nav_frame = tk.Frame(self, bg="#1c1c1e")
        nav_frame.pack(pady=5)
        tk.Button(nav_frame, text="<", command=self.prev_file, bg="#5e5ce6", fg="white", relief="flat").pack(side="left")
        tk.Label(nav_frame, text=self.files[self.current_index], bg="#1c1c1e", fg="white").pack(side="left", padx=10)
        tk.Button(nav_frame, text=">", command=self.next_file, bg="#5e5ce6", fg="white", relief="flat").pack(side="left")
        tk.Button(self, text="MENU", command=self.return_to_menu, bg="#5e5ce6", fg="white", relief="flat").pack(pady=5)

        file_path = os.path.join(self.folder_path.get(), self.files[self.current_index])
        if file_path.endswith(".npy"):
            arr = np.load(file_path)
            img = ImageTk.PhotoImage(Image.fromarray((arr / arr.max() * 255).astype(np.uint8)))
            label = tk.Label(self, image=img, bg="#1c1c1e")
            label.image = img
            label.pack()
        elif file_path.endswith(".png"):
            img = Image.open(file_path)
            img = ImageTk.PhotoImage(img)
            label = tk.Label(self, image=img, bg="#1c1c1e")
            label.image = img
            label.pack()

    def return_to_menu(self):
        self.build_menu()

    def prev_file(self):
        if self.current_index > 0:
            self.current_index -= 1
            self.display_file()

    def next_file(self):
        if self.current_index < len(self.files) - 1:
            self.current_index += 1
            self.display_file()

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
