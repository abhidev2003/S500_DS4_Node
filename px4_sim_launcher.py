#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import subprocess
import threading
import queue
import os
import signal
import time

# Dictionary of command templates with placeholders for dynamic values
COMMAND_TEMPLATES = [
    ("0. Discovery Server", ""), # Command is dynamically injected based on Mode
    ("1. XRCE-DDS Agent", "MicroXRCEAgent udp4 -p {xrce_port}"),
    ("2. PX4 Gazebo", "bash -c 'export PX4_GZ_WORLD={gz_world} && cd ~/PX4-Autopilot && __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia make px4_sitl gz_x500_custom'"),
    ("3. Joy Node", "bash -c 'source /opt/ros/humble/setup.bash && ros2 run joy joy_node'"),
    ("4. Controller Node", "bash -c 'source ~/skypal_ws/install/setup.bash && ros2 run skypal_controller controller_node --ros-args -p is_sim:=True'"),
    ("5. Heart Node (PX4)", "bash -c 'source ~/skypal_ws/install/setup.bash && ros2 run skypal_core heart_node --ros-args -p is_sim:=True'")
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
        
        self.append_text(f"--- Welcome to Skypal Native PX4 Terminal ---\nTarget: {self.name}\n\n[INFO] Component ready for sequenced startup.\n\n")
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
            cmd_template = self.cmd_template
            mode = self.app.mode_var.get()
            
            # --- Dynamic Command Routing based on Mode ---
            if "0. Discovery Server" in self.name:
                if mode == "PURE_SIM":
                    cmd_template = "fastdds discovery -i 0 -l 127.0.0.1 -p 11811"
                else: # Real Flight or RPI SITL
                    # Run the FastDDS Discovery Server LOCALLY on the PC, bound to all interfaces so the Pi can connect
                    cmd_template = f"fastdds discovery -i 0 -l 0.0.0.0 -p 11811"
            
            if mode == "REAL_FLIGHT":
                # In Real Flight, the Micro XRCE-DDS Agent must run on the Pi to talk to the physical Pixhawk
                if "XRCE-DDS Agent" in self.name:
                    # Point the Agent on the Pi BACK to the PC's Tailscale IP
                    cmd_template = f"sshpass -p 'skypal1234' ssh -tt -o StrictHostKeyChecking=no skypal@skyberry \"bash -c 'source /opt/ros/humble/setup.bash && export ROS_DISCOVERY_SERVER=100.122.20.128:11811 && MicroXRCEAgent serial -D /dev/ttyAMA0 -b 921600'\""
                elif "Controller Node" in self.name or "Heart Node" in self.name:
                    cmd_template = cmd_template.replace("is_sim:=True", "is_sim:=False")
                    
            if mode in ["REAL_FLIGHT", "RPI_SITL"]:
                # In both Remote modes, the Heart Node runs on the Pi to simulate/execute over the network
                if "Heart Node" in self.name:
                    is_sim_flag = "False" if mode == "REAL_FLIGHT" else "True"
                    # Point the Heart Node on the Pi BACK to the PC's Tailscale IP
                    cmd_template = f"sshpass -p 'skypal1234' ssh -tt -o StrictHostKeyChecking=no skypal@skyberry \"bash -c 'source /opt/ros/humble/setup.bash && source ~/skypal_ws/install/setup.bash && export ROS_DISCOVERY_SERVER=100.122.20.128:11811 && ros2 run skypal_core heart_node --ros-args -p is_sim:={is_sim_flag}'\""
                
                # The local PC nodes must point to the local 0.0.0.0 server so they can see EACH OTHER
                elif "Joy Node" in self.name or "Controller Node" in self.name or ("XRCE-DDS Agent" in self.name and mode == "RPI_SITL"):
                    cmd_template = "export ROS_DISCOVERY_SERVER=127.0.0.1:11811; " + cmd_template

            # Set local discovery server purely for cleanliness in PURE_SIM mode
            if mode == "PURE_SIM" and ("Joy Node" in self.name or "Controller Node" in self.name or "Heart Node" in self.name or "XRCE-DDS Agent" in self.name):
                cmd_template = "export ROS_DISCOVERY_SERVER=127.0.0.1:11811; " + cmd_template

            cmd_string = cmd_template.format(
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

class PX4SkypalLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Skypal PX4 Native Mission Control")
        self.geometry("1150x750")
        
        # Use a modern theme
        style = ttk.Style()
        if 'clam' in style.theme_names():
            style.theme_use('clam')
            
        style.configure('TButton', font=('Helvetica', 10, 'bold'))
        
        # Header frame
        header = ttk.Frame(self)
        header.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(header, text="Skypal PX4 Native Pipeline", font=('Helvetica', 16, 'bold')).pack(side='left')
        
        btn_frame = ttk.Frame(header)
        btn_frame.pack(side='right')
        ttk.Button(btn_frame, text="▶ Sequence Start", command=self.start_sequence).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="⏹ Stop All Nodes", command=self.stop_all).pack(side='left', padx=5)

        # Configuration Variables
        self.mode_var = tk.StringVar(value="PURE_SIM")
        self.vpn_ip_var = tk.StringVar(value="skyberry") # the Tailscale IP of Skyberry
        self.xrce_port_var = tk.StringVar(value="8888")
        self.gz_world_var = tk.StringVar(value="baylands")

        # Mode Selection Frame
        mode_frame = ttk.LabelFrame(self, text="Operation Mode")
        mode_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Radiobutton(mode_frame, text="💻 Pure software mode", variable=self.mode_var, value="PURE_SIM", command=self.on_mode_change).pack(side='left', padx=15, pady=5)
        ttk.Radiobutton(mode_frame, text="🌐 Raspberry + SITL mode (Tailscale Teleop Test)", variable=self.mode_var, value="RPI_SITL", command=self.on_mode_change).pack(side='left', padx=15, pady=5)
        ttk.Radiobutton(mode_frame, text="🚁 Real flight", variable=self.mode_var, value="REAL_FLIGHT", command=self.on_mode_change).pack(side='left', padx=15, pady=5)


        # Configuration Frame
        config_frame = ttk.LabelFrame(self, text="PX4 Configuration")
        config_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(config_frame, text="VPN Target IP (Skyberry):").grid(row=0, column=0, padx=5, pady=5, sticky='e')
        ttk.Entry(config_frame, textvariable=self.vpn_ip_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(config_frame, text="XRCE-DDS Port:").grid(row=0, column=2, padx=5, pady=5, sticky='e')
        ttk.Entry(config_frame, textvariable=self.xrce_port_var, width=8).grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Label(config_frame, text="Gazebo World:").grid(row=0, column=4, padx=5, pady=5, sticky='e')
        world_cb = ttk.Combobox(config_frame, textvariable=self.gz_world_var, width=15, state="readonly")
        # PX4 exact string names for valid simulation worlds
        world_cb['values'] = ('default', 'empty', 'baylands', 'forest', 'windy', 'maze', 'rover', 'underwater')
        world_cb.grid(row=0, column=5, padx=5, pady=5)
        
        # Ping Frame
        ping_frame = ttk.Frame(config_frame)
        ping_frame.grid(row=1, column=0, columnspan=6, pady=5, sticky='w')
        ttk.Button(ping_frame, text="Ping Skyberry", command=self.ping_skyberry).pack(side='left', padx=5)
        self.ping_status_var = tk.StringVar(value="Status: Unknown")
        ttk.Label(ping_frame, textvariable=self.ping_status_var, font=('Helvetica', 10, 'bold')).pack(side='left', padx=10)
        
        # Network Manager Frame
        net_frame = ttk.LabelFrame(self, text="Skyberry Network Manager")
        net_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(net_frame, text="Active WiFi:").grid(row=0, column=0, padx=5, pady=5, sticky='e')
        self.active_wifi_var = tk.StringVar(value="Unknown")
        ttk.Label(net_frame, textvariable=self.active_wifi_var, font=('Helvetica', 10, 'bold')).grid(row=0, column=1, padx=5, pady=5, sticky='w')
        
        ttk.Button(net_frame, text="Verify Active WiFi", command=self.verify_wifi).grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Label(net_frame, text="Switch to Network:").grid(row=0, column=3, padx=15, pady=5, sticky='e')
        self.target_wifi_var = tk.StringVar(value="JIOPAL")
        wifi_cb = ttk.Combobox(net_frame, textvariable=self.target_wifi_var, width=15, state="readonly")
        wifi_cb['values'] = ('AirFiber-Abhi', 'JIOPAL')
        wifi_cb.grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(net_frame, text="Force Connect", command=self.force_wifi).grid(row=0, column=5, padx=5, pady=5)
        
        # Tabbed layout for terminals
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(expand=True, fill='both', padx=10, pady=5)
        
        self.tabs = []
        for name, template in COMMAND_TEMPLATES:
            tab = ProcessTab(self.notebook, name, template, self)
            self.notebook.add(tab, text=name)
            self.tabs.append(tab)
            
        self.on_mode_change() # initialize disabled states
            
    def ping_skyberry(self):
        self.ping_status_var.set("Status: Pinging...")
        
        def run_ping():
            ip = self.vpn_ip_var.get()
            try:
                # -c 1 (1 packet), -W 2 (2 seconds timeout)
                result = subprocess.run(['ping', '-c', '1', '-W', '2', ip], capture_output=True, text=True)
                if result.returncode == 0:
                    self.after(0, lambda: self.ping_status_var.set(f"Status: ONLINE ({ip})"))
                else:
                    self.after(0, lambda: self.ping_status_var.set(f"Status: OFFLINE ({ip})"))
            except Exception as e:
                self.after(0, lambda err=e: self.ping_status_var.set(f"Status: ERROR ({err})"))
                
        threading.Thread(target=run_ping, daemon=True).start()
        
    def verify_wifi(self):
        self.active_wifi_var.set("Querying...")
        def run_verify():
            ip = self.vpn_ip_var.get()
            cmd = f"sshpass -p 'skypal1234' ssh -o StrictHostKeyChecking=no skypal@{ip} \"echo skypal1234 | sudo -S wpa_cli -i wlan0 status\""
            try:
                result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
                out = result.stdout + "\n" + result.stderr
                ssid = "OFFLINE/NOT FOUND"
                for line in out.splitlines():
                    line = line.strip()
                    if line.startswith("ssid="):
                        ssid = line.split("=")[1].strip()
                        break
                self.after(0, lambda: self.active_wifi_var.set(ssid))
            except Exception:
                self.after(0, lambda: self.active_wifi_var.set("ERROR"))
        threading.Thread(target=run_verify, daemon=True).start()

    def force_wifi(self):
        target = self.target_wifi_var.get()
        if not target: return
        self.active_wifi_var.set(f"Connecting to {target}...")
        
        def run_force():
            ip = self.vpn_ip_var.get()
            # 1. Get List of Networks
            cmd_list = f"sshpass -p 'skypal1234' ssh -o StrictHostKeyChecking=no skypal@{ip} \"echo skypal1234 | sudo -S wpa_cli -i wlan0 list_networks\""
            
            try:
                res = subprocess.run(cmd_list, shell=True, capture_output=True, text=True, timeout=10)
                net_id = None
                for line in res.stdout.splitlines():
                    parts = line.split('\t')
                    if len(parts) >= 2 and parts[1].strip() == target:
                        net_id = parts[0].strip()
                        break
                
                if net_id is not None:
                    # 2. Select the network safely
                    cmd_select = f"sshpass -p 'skypal1234' ssh -o StrictHostKeyChecking=no skypal@{ip} \"echo skypal1234 | sudo -S wpa_cli -i wlan0 select_network {net_id}\""
                    subprocess.run(cmd_select, shell=True, timeout=5)
            except Exception as e:
                pass
                
            self.after(3000, self.verify_wifi)
            
        threading.Thread(target=run_force, daemon=True).start()
            
    def on_mode_change(self):
        # Dynamically disable/enable tabs based on the selected mode
        mode = self.mode_var.get()
        for i, tab in enumerate(self.tabs):
            if "Gazebo" in tab.name:
                self.notebook.tab(i, state=("disabled" if mode == "REAL_FLIGHT" else "normal"))

    def _start_next_in_sequence(self, index=0):
        if index >= len(self.tabs):
            messagebox.showinfo("Sequence Complete", "All Skypal nodes have been dispatched!")
            return
            
        tab = self.tabs[index]
        
        if self.notebook.tab(index, "state") == "disabled":
            self.after(10, lambda: self._start_next_in_sequence(index + 1))
            return
            
        self.notebook.select(tab) # Focus the tab visually
        tab.start_process()
        
        # Delay logic
        delay_ms = 2000
        if "Gazebo" in tab.name:
            selected_world = self.gz_world_var.get()
            if selected_world in ['empty', 'default']:
                delay_ms = 8000
            elif selected_world in ['windy', 'underwater']:
                delay_ms = 12000
            else:
                delay_ms = 18000
        elif "Agent" in tab.name:
            delay_ms = 4000
            
        self.after(delay_ms, lambda: self._start_next_in_sequence(index + 1))
            
    def start_sequence(self):
        for tab in self.tabs:
            if tab.process is not None:
                messagebox.showwarning("Sequence Error", "Some nodes are already running. Please Stop All Nodes first.")
                return
                
        if self.mode_var.get() == "RPI_SITL":
            def check_eth0():
                ip = self.vpn_ip_var.get()
                cmd = f"sshpass -p 'skypal1234' ssh -o StrictHostKeyChecking=no skypal@{ip} \"cat /sys/class/net/eth0/carrier\""
                try:
                    res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
                    has_cable = (res.stdout.strip() == "1")
                except Exception:
                    has_cable = False

                if has_cable:
                    self.after(0, lambda: self._start_next_in_sequence(0))
                else:
                    def ask_user():
                        msg = ("No physical RJ45 cable detected between your laptop and the Raspberry Pi.\n\n"
                               "This will cause signal double latency (Laptop -> Pi -> Laptop) during this SITL simulation over the VPN.\n\n"
                               "Do you want to start the simulation anyway?")
                        if messagebox.askyesno("Hardware Latency Warning", msg):
                            self._start_next_in_sequence(0)
                    self.after(0, ask_user)
            # Run the SSH check in a background thread so the GUI doesn't freeze
            threading.Thread(target=check_eth0, daemon=True).start()
        else:
            self._start_next_in_sequence(0)
            
    def stop_all(self):
        for tab in self.tabs:
            tab.stop_process()

if __name__ == "__main__":
    app = PX4SkypalLauncher()
    app.protocol("WM_DELETE_WINDOW", lambda: [app.stop_all(), app.destroy()])
    app.mainloop()
