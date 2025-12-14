#!/usr/bin/env python3
"""
ChipuRobo Path Editor - GUI tool for creating robot paths
Moved from tools/path_editor/editor.py and updated for new structure
"""

import tkinter as tk
from tkinter import filedialog, messagebox
import json
import os


class PathEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("ChipuRobo Path Editor")
        
        # Canvas for path creation
        self.canvas = tk.Canvas(root, width=1000, height=700, bg="white", 
                               relief="solid", borderwidth=2)
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)
        
        self.draw_grid()
        self.points = []
        
        # Control panel
        self.create_control_panel()
        
        # Bind events
        self.canvas.bind("<Button-1>", self.add_point)
        
    def draw_grid(self):
        """Draw grid on canvas"""
        for i in range(0, 1000, 50):
            self.canvas.create_line(i, 0, i, 700, fill="lightgray", width=1)
        for i in range(0, 700, 50):
            self.canvas.create_line(0, i, 1000, i, fill="lightgray", width=1)
            
    def create_control_panel(self):
        """Create control panel"""
        frame = tk.Frame(self.root)
        frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10)
        
        tk.Label(frame, text="Path Editor", font=("Arial", 16, "bold")).pack(pady=10)
        
        tk.Button(frame, text="💾 Save Path", command=self.save_path,
                 bg="lightgreen", width=15).pack(pady=5)
        tk.Button(frame, text="📁 Load Path", command=self.load_path,
                 bg="lightblue", width=15).pack(pady=5) 
        tk.Button(frame, text="🗑️ Clear", command=self.clear_path,
                 bg="lightcoral", width=15).pack(pady=5)
        
        tk.Label(frame, text="Waypoints:", font=("Arial", 12, "bold")).pack(pady=(20,5))
        self.listbox = tk.Listbox(frame, width=25, height=20)
        self.listbox.pack(pady=5)
        
        self.status = tk.Label(frame, text="Click to add waypoints", fg="blue")
        self.status.pack(pady=10)
        
    def add_point(self, event):
        """Add waypoint at click position"""
        x, y = event.x, event.y
        self.points.append({'x': x, 'y': y})
        
        # Draw point
        self.canvas.create_oval(x-5, y-5, x+5, y+5, fill="red", outline="darkred")
        self.canvas.create_text(x+10, y-10, text=str(len(self.points)), fill="red")
        
        # Draw line to previous point
        if len(self.points) > 1:
            prev = self.points[-2]
            self.canvas.create_line(prev['x'], prev['y'], x, y, fill="blue", width=2)
            
        # Update list
        self.listbox.insert(tk.END, f"{len(self.points)}: ({x}, {y})")
        self.status.config(text=f"Added point {len(self.points)}")
        
    def save_path(self):
        """Save path to JSON file"""
        if not self.points:
            messagebox.showwarning("Warning", "No path to save!")
            return
            
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
            initialdir="../deploy/pathplanner/paths"
        )
        
        if filename:
            # Convert to field coordinates (simplified)
            waypoints = []
            for i, point in enumerate(self.points):
                waypoints.append({
                    'x': (point['x'] - 20) / 50.0,  # Convert pixels to feet
                    'y': (700 - point['y'] - 20) / 50.0,
                    'velocity': 2.0,
                    'heading': 0.0
                })
                
            path_data = {
                'name': os.path.basename(filename).replace('.json', ''),
                'waypoints': waypoints,
                'created_by': 'ChipuRobo Path Editor'
            }
            
            with open(filename, 'w') as f:
                json.dump(path_data, f, indent=2)
                
            self.status.config(text=f"Saved: {os.path.basename(filename)}")
            
    def load_path(self):
        """Load path from JSON file"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json")],
            initialdir="../deploy/pathplanner/paths"
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                    
                self.clear_path()
                
                if 'waypoints' in data:
                    for wp in data['waypoints']:
                        # Convert field coordinates back to pixels
                        x = int(wp['x'] * 50 + 20)
                        y = int(700 - wp['y'] * 50 - 20)
                        
                        # Simulate click to add point
                        class FakeEvent:
                            def __init__(self, x, y):
                                self.x = x
                                self.y = y
                                
                        self.add_point(FakeEvent(x, y))
                        
                self.status.config(text=f"Loaded: {os.path.basename(filename)}")
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load path: {e}")
                
    def clear_path(self):
        """Clear all waypoints"""
        self.canvas.delete("all")
        self.draw_grid()
        self.points.clear()
        self.listbox.delete(0, tk.END)
        self.status.config(text="Path cleared")


def main():
    """Launch path editor"""
    root = tk.Tk()
    root.resizable(False, False)
    
    # Center window
    root.geometry("1300x750+100+50")
    
    app = PathEditor(root)
    
    print("🎯 ChipuRobo Path Editor started")
    print("   Click on canvas to create waypoints")
    print("   Paths saved to: deploy/pathplanner/paths/")
    
    root.mainloop()


if __name__ == "__main__":
    main()