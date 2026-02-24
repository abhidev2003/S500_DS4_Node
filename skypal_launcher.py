#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import subprocess
import threading
import queue
import os
import signal
import time

# Dictionary of command templates with placeholders for dynamic IPs and Ports
# Order matters here! This defines the ordered boot sequence.
COMMAND_TEMPLATES = [
    ("1. Fast DDS VPN", "bash -c '~/skypal_ws/fastdds_setup.sh'"),
    ("2. XRCE-DDS Agent", "bash -c 'export ROS_DISCOVERY_SERVER=\"{vpn_ip}:{vpn_port}\"; MicroXRCEAgent udp4 -p {xrce_port} -i {rj45_ip}'"),
    ("3. PX4 Gazebo", "bash -c 'export PX4_GZ_WORLD={gz_world} && cd ~/PX4-Autopilot && make px4_sitl gz_x500'"),
    ("4. Joy Node", "bash -c 'export ROS_DISCOVERY_SERVER=\"{vpn_ip}:{vpn_port}\"; source /opt/ros/humble/setup.bash && ros2 run joy joy_node'"),
    ("5. Controller (PC)", "bash -c 'export ROS_DISCOVERY_SERVER=\"{vpn_ip}:{vpn_port}\"; source ~/skypal_ws/install/setup.bash && ros2 run skypal_controller controller_node'"),
    ("6. Heart (Pi)", "sshpass -p 'skypal1234' ssh -tt skypal@skyberry 'export ROS_DISCOVERY_SERVER=\"{vpn_ip}:{vpn_port}\"; source ~/skypal_ws/install/setup.bash && ros2 run skypal_core heart_node'")
]

class ProcessTab(ttk.Frame):
    def __init__(self, parent, name, cmd_template, app_ref):
        super().__init__(parent)
        self.name = name
        self.cmd_template = cmd_template
        self.app = app_ref
        self.process = None
        self.q = queue.Queue()
        
        # Build UI Elements
        self.text_area = scrolledtext.ScrolledText(self, state='disabled', bg='#1e1e1e', fg='#00ff00', font=('Courier', 10))
        self.text_area.pack(expand=True, fill='both')
        
        self.input_frame = ttk.Frame(self)
        self.input_frame.pack(fill='x', pady=5, padx=5)
        
        ttk.Label(self.input_frame, text="Terminal Input:").pack(side='left')
        self.cmd_entry = ttk.Entry(self.input_frame)
        self.cmd_entry.pack(side='left', expand=True, fill='x', padx=5)
        self.cmd_entry.bind("<Return>", self.send_input)
        
        self.send_btn = ttk.Button(self.input_frame, text="Send", command=self.send_input)
        self.send_btn.pack(side='left', padx=2)
        
        self.start_btn = ttk.Button(self.input_frame, text="Start Instance", command=self.start_process)
        self.start_btn.pack(side='left', padx=2)
        
        self.stop_btn = ttk.Button(self.input_frame, text="Stop Instance", command=self.stop_process, state='disabled')
        self.stop_btn.pack(side='left', padx=2)

        self.clear_btn = ttk.Button(self.input_frame, text="Clear Logs", command=self.clear_text)
        self.clear_btn.pack(side='left', padx=2)
        
        self.append_text(f"--- Welcome to Skypal Terminal ---\nTarget: {self.name}\n\n[INFO] Component ready for sequenced startup.\n\n")
        self.after(100, self.update_text)

    def append_text(self, text):
        self.text_area.configure(state='normal')
        self.text_area.insert(tk.END, text)
        self.text_area.see(tk.END)
        self.text_area.configure(state='disabled')
        
    def enqueue_output(self, out, queue):
        try:
            for line in iter(out.readline, b''):
                queue.put(line.decode('utf-8', errors='replace'))
        except Exception:
            pass
        out.close()

    def update_text(self):
        try:
            while True:
                line = self.q.get_nowait()
                self.append_text(line)
        except queue.Empty:
            pass
        
        if self.process and self.process.poll() is not None:
            self.stop_process()
            
        self.after(100, self.update_text)

    def start_process(self):
        if self.process is None:
            # Format the command template with current network configuration values from the main app
            cmd_string = self.cmd_template.format(
                vpn_ip=self.app.vpn_ip_var.get(),
                vpn_port=self.app.vpn_port_var.get(),
                rj45_ip=self.app.rj45_ip_var.get(),
                xrce_port=self.app.xrce_port_var.get(),
                gz_world=self.app.gz_world_var.get()
            )
            
            self.append_text(f"\n>>> Executing Command: {self.name} <<<\n$ {cmd_string}\n\n")
            # We use setsid so we can kill the whole bash process group including children
            self.process = subprocess.Popen(
                cmd_string,
                shell=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                preexec_fn=os.setsid
            )
            
            self.thread = threading.Thread(target=self.enqueue_output, args=(self.process.stdout, self.q))
            self.thread.daemon = True
            self.thread.start()
            
            self.start_btn.configure(state='disabled')
            self.stop_btn.configure(state='normal')

    def stop_process(self):
        if self.process:
            self.append_text("\n>>> Terminating Process... <<<\n")
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            self.process = None
            self.start_btn.configure(state='normal')
            self.stop_btn.configure(state='disabled')

    def send_input(self, event=None):
        if self.process and self.process.stdin:
            cmd = self.cmd_entry.get() + '\n'
            try:
                self.process.stdin.write(cmd.encode('utf-8'))
                self.process.stdin.flush()
                self.append_text(f">> {cmd}")
                self.cmd_entry.delete(0, tk.END)
            except BrokenPipeError:
                self.append_text("\n[Error: Process closed. Cannot receive input]\n")

    def clear_text(self):
        self.text_area.configure(state='normal')
        self.text_area.delete('1.0', tk.END)
        self.text_area.configure(state='disabled')

class SkypalLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Skypal Mission Control Launcher")
        self.geometry("1150x750")
        
        # Use a modern theme
        style = ttk.Style()
        if 'clam' in style.theme_names():
            style.theme_use('clam')
            
        style.configure('TButton', font=('Helvetica', 10, 'bold'))
        
        # Header frame
        header = ttk.Frame(self)
        header.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(header, text="Skypal Hardware-in-the-Loop Simulation Console", font=('Helvetica', 16, 'bold')).pack(side='left')
        
        btn_frame = ttk.Frame(header)
        btn_frame.pack(side='right')
        ttk.Button(btn_frame, text="▶ Sequence Start", command=self.start_sequence).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="⏹ Stop All Nodes", command=self.stop_all).pack(side='left', padx=5)

        # Configuration Variables
        self.vpn_ip_var = tk.StringVar(value="100.122.20.128")
        self.vpn_port_var = tk.StringVar(value="11811")
        self.rj45_ip_var = tk.StringVar(value="192.168.100.1")
        self.xrce_port_var = tk.StringVar(value="8888")
        self.gz_world_var = tk.StringVar(value="default")

        # Configuration Frame
        config_frame = ttk.LabelFrame(self, text="Simulation & Network Configuration")
        config_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(config_frame, text="VPN IP:").grid(row=0, column=0, padx=5, pady=5, sticky='e')
        ttk.Entry(config_frame, textvariable=self.vpn_ip_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(config_frame, text="VPN Port:").grid(row=0, column=2, padx=5, pady=5, sticky='e')
        ttk.Entry(config_frame, textvariable=self.vpn_port_var, width=8).grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(config_frame, text="RJ45 Target IP:").grid(row=0, column=4, padx=5, pady=5, sticky='e')
        ttk.Entry(config_frame, textvariable=self.rj45_ip_var, width=15).grid(row=0, column=5, padx=5, pady=5)

        ttk.Label(config_frame, text="XRCE-DDS Port:").grid(row=0, column=6, padx=5, pady=5, sticky='e')
        ttk.Entry(config_frame, textvariable=self.xrce_port_var, width=8).grid(row=0, column=7, padx=5, pady=5)
        
        ttk.Label(config_frame, text="Gazebo World:").grid(row=0, column=8, padx=5, pady=5, sticky='e')
        world_cb = ttk.Combobox(config_frame, textvariable=self.gz_world_var, width=15, state="readonly")
        # PX4 exact string names for valid simulation worlds
        world_cb['values'] = ('default', 'empty', 'baylands', 'forest', 'windy', 'maze', 'rover', 'underwater')
        world_cb.grid(row=0, column=9, padx=5, pady=5)
        
        # Tabbed layout for terminals
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(expand=True, fill='both', padx=10, pady=5)
        
        self.tabs = []
        for name, template in COMMAND_TEMPLATES:
            tab = ProcessTab(self.notebook, name, template, self)
            self.notebook.add(tab, text=name)
            self.tabs.append(tab)
            
    def _start_next_in_sequence(self, index=0):
        if index >= len(self.tabs):
            messagebox.showinfo("Sequence Complete", "All Skypal nodes have been dispatched!")
            return
            
        tab = self.tabs[index]
        self.notebook.select(tab) # Focus the tab visually
        tab.start_process()
        
        # Delay logic based on what was just started
        delay_ms = 2000 # Default delay
        if "Gazebo" in tab.name:
            # Dynamically adjust the boot delay based on the complexity of the selected world
            selected_world = self.gz_world_var.get()
            if selected_world in ['empty', 'default']:
                delay_ms = 8000   # 8 Seconds for simple worlds
            elif selected_world in ['windy', 'underwater']:
                delay_ms = 12000  # 12 Seconds for medium worlds
            else:
                delay_ms = 18000  # 18 Seconds for heavy worlds (baylands, forest, maze, rover)
        elif "Agent" in tab.name:
            delay_ms = 3000 # Give DDS Agent a moment to bind ports
            
        self.after(delay_ms, lambda: self._start_next_in_sequence(index + 1))
            
    def start_sequence(self):
        # Prevent double-starting the sequence
        for tab in self.tabs:
            if tab.process is not None:
                messagebox.showwarning("Sequence Error", "Some nodes are already running. Please Stop All Nodes first.")
                return
        
        self._start_next_in_sequence(0)
            
    def stop_all(self):
        for tab in self.tabs:
            tab.stop_process()

if __name__ == "__main__":
    app = SkypalLauncher()
    # Ensure all child subprocesses are killed when closing the GUI
    app.protocol("WM_DELETE_WINDOW", lambda: [app.stop_all(), app.destroy()])
    app.mainloop()
