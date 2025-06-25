import os
import tkinter as tk

class FolderExplorer(tk.Toplevel):
    def __init__(self, master, on_select_callback, initial_dir="."):
        super().__init__(master)
        self.title("Select Folder")
        self.configure(bg="#d6ccff")
        self.geometry("600x500")
        self.on_select = on_select_callback

        self.current_path = os.path.abspath(initial_dir)
        self.selected_folder = None
        self.back_stack = []
        self.forward_stack = []

        nav_frame = tk.Frame(self, bg="#d6ccff")
        nav_frame.pack(fill="x", padx=10, pady=(10, 0))

        tk.Button(nav_frame, text="Back", command=self.go_back, bg="#bcaaff", fg="black", relief="flat").pack(side="left")
        tk.Button(nav_frame, text="Forward", command=self.go_forward, bg="#bcaaff", fg="black", relief="flat").pack(side="left", padx=(5, 0))

        self.path_label = tk.Label(self, text=self.current_path, bg="#d6ccff", fg="#2c2c2e", font=("Arial", 12, "bold"))
        self.path_label.pack(fill="x", padx=10, pady=(5, 5))

        self.list_frame = tk.Frame(self, bg="#d6ccff")
        self.list_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.up_button = tk.Button(self.list_frame, text="Go Up (..)", command=self.go_up, bg="#c2b8ff", fg="black", relief="flat")
        self.up_button.pack(fill="x", pady=(0, 5))

        self.folder_listbox = tk.Listbox(self.list_frame, bg="#f1edff", fg="black", font=("Arial", 11),
                                         activestyle='none', selectbackground="#b8aaff")
        self.folder_listbox.pack(fill="both", expand=True)
        self.folder_listbox.bind("<<ListboxSelect>>", self.on_select_change)
        self.folder_listbox.bind("<Double-Button-1>", self.on_folder_double_click)

        button_frame = tk.Frame(self, bg="#d6ccff")
        button_frame.pack(pady=(10, 5))
        tk.Button(button_frame, text="Select", command=self.select_folder, bg="#5e5ce6", fg="white").pack(side="left", padx=10)

        self.selection_label = tk.Label(self, text="Selected: (none)", bg="#d6ccff", fg="#3b3b3b",
                                        font=("Arial", 10, "italic"), anchor="w", justify="left")
        self.selection_label.pack(fill="x", padx=10, pady=(0, 10))

        self.populate_folder_list()

        self.last_hover_index = None
        self.folder_listbox.bind("<Motion>", self.on_mouse_hover)

    def populate_folder_list(self):
        self.folder_listbox.delete(0, tk.END)
        try:
            folders = [f for f in os.listdir(self.current_path) if os.path.isdir(os.path.join(self.current_path, f))]
            for folder in sorted(folders):
                self.folder_listbox.insert(tk.END, folder)
        except PermissionError:
            self.folder_listbox.insert(tk.END, "<Access Denied>")
        self.path_label.config(text=self.current_path)
        self.last_hover_index = None

    def go_up(self):
        parent = os.path.dirname(self.current_path)
        if parent != self.current_path:
            self.back_stack.append(self.current_path)
            self.current_path = parent
            self.forward_stack.clear()
            self.populate_folder_list()

    def go_back(self):
        if self.back_stack:
            self.forward_stack.append(self.current_path)
            self.current_path = self.back_stack.pop()
            self.populate_folder_list()

    def go_forward(self):
        if self.forward_stack:
            self.back_stack.append(self.current_path)
            self.current_path = self.forward_stack.pop()
            self.populate_folder_list()

    def on_select_change(self, event):
        selected = self.folder_listbox.get(tk.ACTIVE)
        full_path = os.path.join(self.current_path, selected)
        if os.path.isdir(full_path):
            self.selected_folder = full_path
            self.selection_label.config(text=f"Selected: {self.selected_folder}")

    def on_folder_double_click(self, event):
        selected = self.folder_listbox.get(tk.ACTIVE)
        next_path = os.path.join(self.current_path, selected)
        if os.path.isdir(next_path):
            self.selected_folder = next_path
            self.selection_label.config(text=f"Selected: {self.selected_folder}")
            self.back_stack.append(self.current_path)
            self.current_path = next_path
            self.forward_stack.clear()
            self.populate_folder_list()

    def select_folder(self):
        if self.selected_folder:
            self.on_select(self.selected_folder)
        self.destroy()

    def on_mouse_hover(self, event):
        index = self.folder_listbox.nearest(event.y)

        if index < 0 or index >= self.folder_listbox.size():
            return  # Ignore if the mouse is outside valid items

        if index != self.last_hover_index:
            if self.last_hover_index is not None and 0 <= self.last_hover_index < self.folder_listbox.size():
                self.folder_listbox.itemconfig(self.last_hover_index, bg="#f1edff")

            self.folder_listbox.itemconfig(index, bg="#e2dbff")
            self.last_hover_index = index
