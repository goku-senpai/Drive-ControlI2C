import numpy as np
import serial
from tkinter import *
from tkinter import Canvas
from tkinter import ttk
from PIL import ImageTk, Image
import math
import serial.tools.list_ports
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
from collections import deque


debug = False

class Window(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.after_id = None  # Store the ID for the periodic update
        self.MotorHelpSelection = None
        self.position_input = None
        self.start_time = time.time()  # Initialize start_time
        self.master = master
        self.dark_mode = False
        self.serial_port = None
        self.receive_data_flag = False
        self.motor_stop_flag = False

        self.init_window()
        self.connected = False
        self.bFreeWheel = True
        self.populate_combo_box()
        self.connected = False  # Initialize the connection status
        self.dc_help_text = "To calculate the PID gains for a DC motor, you need to measure the motor's speed and current while it's running under load. Then, you can use the following formula:\n\nL = Kt / R\n\nWhere L is the PID gain, Kt is the torque constant, and R is the armature resistance."
        self.stepper_help_text = "To calculate the PID gains for a stepper motor, you need to measure the motor's torque and speed while it's running under load. Then, you can use the following formula:\n\nL = (Kt / R) * (60 / S)\n\nWhere L is the PID gain, Kt is the torque constant, R is the winding resistance, and S is the stepping rate in steps per second."
        self.servo_help_text = "To calculate the PID gains for a servo motor, you need to measure the motor's velocity and torque while it's running under load. Then, you can use the following formula:\n\nL = Kt / Kb\n\nWhere L is the PID gain, Kt is the torque constant, and Kb is the back-emf constant."
        self.zn_help_text = "The (proportional) gain, K p is increased (from zero) until it reaches the ultimate gain K u, at which the output of the control loop has stable and consistent oscillations. K u K_{u} and the oscillation period T u T_{u} are then used to set the P, I, and D gains depending on the type of controller used and behaviour desired. Classic PID: K_p:0.6xK_u; Ki:1.2x K_u/T_u; K_p:0.075xK_uxT_u"
        self.help_label.configure(text="Choose what help you need")

        # Initialize the previous position and time value
        self.previous_position = 0
        self.previous_pos = 0
        self.previous_x_value = 0

        # Initialize Matplotlib plot with initial data
        self.fig, self.ax = plt.subplots(figsize=(4, 3), dpi=100)
        self.x_values = []
        self.smoothed_y_values = []
        self.y_values = []
        self.line, = self.ax.plot([], [])
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().grid(row=9, column=3, columnspan=2, padx=10, pady=10)
        self.plotting_started = False
        self.data_buffer = deque(maxlen=100)
        self.update_plot_periodically()




    def set_needle_position(self, position):
        self.gauge.set_needle_position(position)

    def init_window(self):
        self.master.title("Motor Controller GUI")
        self.value = IntVar()
        self.value.set(0)

        # Add a Windows drop-down menu
        menu = Menu(self.master)

        self.master.config(menu=menu)
        self.master.configure(background="#474757")  # set background to #474757

        ########################################################################################
        file = Menu(menu)
        file.add_command(label='Exit', command=self.client_exit)
        menu.add_cascade(label='File', menu=file)

        ########################################################################################
        # Add a label and drop-down menu for the control mode
        control_label = Label(self.master, text="Control Mode:", fg="white",
                              bg="#474757")  # set foreground to white and background to #474757
        control_label.grid(row=0, column=0, padx=10, pady=10)

        self.control_var = StringVar()
        self.control_var.set("Speed")
        self.control_dropdown = OptionMenu(self.master, self.control_var, "Speed", "Position")
        self.control_dropdown.config(width=15, bg="gray", fg="white")  # set background to gray and foreground to white
        self.control_dropdown.grid(row=0, column=1, padx=10, pady=10)

        # Add a slider for position control and a text field for numerical input
        self.position_slider = ttk.Scale(self.master, from_=0, to=180, orient=HORIZONTAL, length=200, variable=self.value, command=self.update_input)
        self.position_slider.grid(row=1, column=0, padx=10, pady=10)
        self.position_slider_label1 = Label(self.master, text="Use slider or textfield to set v or pos:", fg="white", bg="#474757")
        self.position_slider_label1.grid(row=1, column=1, padx=10, pady=10)
        self.position_input = Entry(self.master, bg="gray", fg="white")
        self.position_input.grid(row=2, column=0, padx=10, pady=0)
        self.position_input.insert(0, self.value.get())
        self.position_input.bind("<Return>", self.update_slider)

        # Add text fields for KP and Ki
        self.kp_label = Label(self.master, text="KP:", fg="white",
                              bg="#474757")  # set foreground to white and background to #474757
        self.kp_label.grid(row=3, column=0, padx=10, pady=10)
        self.kp_input = Entry(self.master, bg="gray", fg="white")  # set background to gray and foreground to white
        self.kp_input.grid(row=3, column=1, padx=10, pady=10)
        self.kp_input.insert(0, "1.0")

        self.ki_label = Label(self.master, text="Ki:", fg="white",
                              bg="#474757")  # set foreground to white and background to #474757
        self.ki_label.grid(row=4, column=0, padx=10, pady=10)
        self.ki_input = Entry(self.master, bg="gray", fg="white")  # set background to gray and foreground to white
        self.ki_input.grid(row=4, column=1, padx=10, pady=10)
        self.ki_input.insert(0, "0.0")

        self.kd_label = Label(self.master, text="Kd:", fg="white",
                              bg="#474757")  # set foreground to white and background to #474757
        self.kd_label.grid(row=5, column=0, padx=10, pady=10)
        self.kd_input = Entry(self.master, bg="gray", fg="white")
        self.kd_input.grid(row=5, column=1, padx=10, pady=10)
        self.kd_input.insert(0, "0.0")

        # Add a button to connect to the selected device
        self.connect_button = ttk.Button(self.master, text="Connect", command=self.connect_to_device)
        self.connect_button.grid(row=6, column=1, padx=10, pady=10)

        # Add a button to start the motor
        self.start_button = ttk.Button(self.master, text="Send Data", command=self.start_motor)
        self.start_button.grid(row=7, column=1, padx=10, pady=10)

        # Add a label for the connection status
        self.connection_status = Label(self.master, text="Not Connected")
        self.connection_status.grid(row=7, column=0, padx=10, pady=10)

        self.stop_button = ttk.Button(self.master, text="Stop Plot", command=self.stop_motor)
        self.stop_button.grid(row=8, column=1, padx=10, pady=10)

        self.clear_button = ttk.Button(self.master, text="clear Plot", command=self.clear_all)
        self.clear_button.grid(row=8, column=3, padx=10, pady=10)


        # Add a button to toggle dark mode
        self.dark_mode_button = ttk.Button(self.master, text="toggle Dark Mode", command=self.toggle_dark_mode)
        self.dark_mode_button.grid(row=8, column=0, padx=10, pady=(10, 20))

        # self.send_button = ttk.Button(self.master, text="send data", command=self.send_data())
        # self.send_button.grid(row=9, column=1, padx=10, pady=10)

        # Add a canvas for the gauge display
        self.gauge_canvas = Canvas(self.master, width=300, height=300)
        self.gauge = Gauge(self.master, width=300, height=300)
        self.gauge.grid(row=9, column=0, columnspan=2, padx=10, pady=10)
        self.x_values = np.linspace(0, 0, 1000)
        self.y_values = np.linspace(0, 0, 1000)

        # add i-pel LOGO
        self.img = ImageTk.PhotoImage(Image.open("logo_ipel.jpg"))
        self.label = Label(self.master, image=self.img, width=400, height=130)
        self.label.grid(row=0, column=3, padx=10, pady=30)


        #####HELP MENUE:
        # Create the drop-down menu
        self.motor_type = ttk.Combobox(self.master, values=["NONE", "DC", "Stepper", "Servo", "Ziegler–Nichols"])
        self.motor_type.grid(row=4, column=3, padx=10, pady=10)

        # Create the help label
        self.help_label = Label(self.master, wraplength=400, justify="left")
        self.help_label.grid(row=1, column=3, padx=10, pady=10)

        # Create the "Show Help" button
        self.help_button = Button(self.master, text="Show Help", command=self.show_help_text)
        self.help_button.grid(row=2, column=3, padx=10, pady=10)


    def client_exit(self):
        exit()

    def populate_combo_box(self):
        # Populate the combo box with available serial ports
        self.ports = serial.tools.list_ports.comports()
        self.port_names = []
        for port in self.ports:
            self.port_names.append(port.device)
        self.port_var = StringVar()
        if self.port_names:
            self.port_var.set(self.port_names[0])
            self.port_dropdown = OptionMenu(self.master, self.port_var, self.port_names[0], *self.port_names)
        else:
            self.port_var.set("No Ports Found")
            self.port_dropdown = OptionMenu(self.master, self.port_var, "No Ports Found")
        self.port_dropdown.config(width=15)
        self.port_dropdown.grid(row=6, column=0, padx=10, pady=10)

    def update_slider(self, event):
        try:
            value = float(self.position_input.get())
            self.value.set(value)
        except ValueError:
            pass

    def update_input(self, value):
        self.position_input.delete(0, END)
        self.position_input.insert(0, self.value.get())

    def toggle_connection(self):
        if not self.connected:
            baud_rate = 115200
            port_name = self.port_var.get()
            if port_name == "No Ports Found":
                self.populate_combo_box()
                port_name = self.port_var.get()
            try:
                self.serial_port = serial.Serial(port_name, baud_rate, timeout=1)
                self.connected = True
            except:
                self.connected = False

            if self.connected:
                self.connect_button["text"] = "Disconnect"  # Change button text to "Disconnect"
                self.start_button["state"] = NORMAL
                self.connection_status.config(text="Connected", fg="green")
                t = threading.Thread(target=self.update_plot)
                t.start()
        else:
            if self.serial_port:
                self.serial_port.close()
            self.connected = False
            self.connect_button["text"] = "Connect"  # Change button text back to "Connect"
            self.start_button["state"] = DISABLED
            self.connection_status.config(text="Not Connected", fg="red")

    def connect_to_device(self):
        self.toggle_connection()

    def clear_gauge(self):
        # Reset the gauge to its initial position
        self.gauge.set_needle_position(0)

    def clear_plot(self):
        self.x_values = []
        self.y_values = []
        self.smoothed_y_values = []
        self.line.set_data([], [])
        self.ax.relim()
        self.ax.autoscale_view()

        # Set the y-axis limits to start from 0
        min_limit = 0
        max_limit = 360  # You can adjust this value as needed
        self.ax.set_ylim(min_limit, max_limit)

        self.canvas.draw_idle()
        self.data_buffer.clear()
        self.previous_time = time.time()


    def clear_all(self):
        # Stop receiving data continuously
        self.receive_data_flag = False
        # Stop receiving data continuously
        self.clear_plot()
        self.clear_gauge()
        self.start_time = time.time()  # Initialize start_time

    def stop_motor(self):
        command = ""
        if self.receive_data_flag :
            # Stop receiving data continuously
            self.receive_data_flag = False
            self.motor_stop_flag = True
            self.start_button.config(text="Start Motor")
        command += "0, 0, 0, 0, 0, 0,"
        if self.serial_port:
            if len(command) <= 100:
                self.serial_port.write((command).encode('utf-8'))
                print("Data Sent:", command)


    def start_motor(self):
        self.motor_stop_flag = False
        command = ""
        command += "X"
        # get params
        control_mode = self.control_var.get()
        if control_mode == "Speed":
            self.bFreewheel = True
            command += "1,"
        else:
            self.bFreeWheel = False
            command += "0,"

        kp = self.kp_input.get()
        ki = self.ki_input.get()
        kd = self.kd_input.get()

        gains = "{:.8f},{:.8f},{:.8f}".format(float(kp), float(ki), float(kd))
        command += gains

        setpoint = self.position_input.get()
        command += ",{:.8f}".format(float(setpoint))

        if self.motor_stop_flag:
            command += "1,"
        else:
            command += "0,"
        command += "\n"
        print("command:", command)

        # Clear the gauge and plot before sending new data
        self.clear_gauge()
        # Clear the lists to reset x and y values
        self.x_values = []
        self.y_values = []
        self.smoothed_y_values = []
        self.clear_plot()
        self.plotting_started = True

        # Start receiving data continuously
        self.receive_data_flag = True

        # Send the command
        if self.serial_port:
            if len(command) <= 100:
                self.serial_port.write((command).encode('utf-8'))
                print("Data Sent:", command)
                self.start_time = time.time()  # Initialize start_time
        else:
                print("Command exceeds 100 bytes, sending truncated data.")
                self.serial_port.write((command[:99]).encode('utf-8'))
                self.start_time = time.time()  # Initialize start_time
        self.update_plot()

    def update_plot_periodically(self):
        # Schedule the update_plot function to run periodically
        self.update_plot()
        self.master.after(60, self.update_plot_periodically)

    def update_plot(self):
        try:
            if self.serial_port and self.receive_data_flag:
                data = self.serial_port.readline().decode('utf-8').strip()
                values = data.split(',')
                #time.sleep(0.01)
                print(data)

                if data and len(values) >= 7:
                    self.y_value = float(values[6])
                    self.x_value = time.time() - self.start_time

                    self.x_values.append(self.x_value)
                    self.y_values.append(self.y_value)

                    smoothed_value = np.mean(self.y_values)
                    self.smoothed_y_values.append(smoothed_value)

                    if len(self.x_values) > 100:
                        self.x_values.pop(0)
                        self.y_values.pop(0)
                        self.smoothed_y_values.pop(0)

                    self.line.set_data(self.x_values, self.smoothed_y_values)
                    self.ax.relim()
                    self.ax.autoscale_view()
                    self.canvas.draw_idle()

                    self.time_interval = self.x_value - self.previous_time
                    if self.time_interval > 0:
                        speed = (self.y_value - self.data_buffer[-1]) / self.time_interval
                    else:
                        speed = 0

                    self.gauge.set_needle_position(speed)
                    self.previous_time = self.x_value
                    self.data_buffer.append(self.y_value)

        except UnicodeDecodeError:
            pass
        except ValueError:
            pass
        except serial.serialutil.PortNotOpenError:
            print("Disconnected")
            self.plotting_started = False
            self.connection_status.config(text="Disconnected", fg="red")

    def toggle_dark_mode(self):
        self.dark_mode = not self.dark_mode
        self.init_window()
        if self.dark_mode:
            self.master.configure(background="#474757")
            # self.master.configure(fg="white")
        else:
            self.master.configure(background="light grey")
            # self.master.configure(fg="#474757")

    def show_help_text(self):
        MotorHelpSelection = self.motor_type.get()
        if MotorHelpSelection == "DC":
            self.help_label.configure(text=self.dc_help_text)
        elif MotorHelpSelection == "Stepper":
            self.help_label.configure(text=self.stepper_help_text)
        elif MotorHelpSelection == "Servo":
            self.help_label.configure(text=self.servo_help_text)
        elif MotorHelpSelection == "Ziegler–Nichols":
            self.help_label.configure(text=self.zn_help_text)
        else:
            self.help_label.configure(text="Choose what help you need")

    def client_exit(self):
    # Cancel the periodic update before exiting
        if self.after_id:
            self.master.after_cancel(self.after_id)
        exit()


class Gauge(Canvas):
    def __init__(self, master=None, width=500, height=400):
        Canvas.__init__(self, master, width=width, height=height, bg='#474757')
        self.width = width
        self.height = height
        self.create_oval(10, 10, self.width - 10, self.height - 10, fill='#474757', outline='#474757')
        self.create_oval(9, 9, self.width - 9, self.height - 9, fill='white', outline='#474757')
        self.needle = self.create_line(0, 0, 0, 0, width=5, fill="#FFC107")

        # Create a label to display the speed value
        self.speed_label = Label(self, text="", font=("Helvetica", 12), fg="white", bg="#474757")
        self.speed_label.place(relx=0.5, rely=0.8, anchor=CENTER)

    def set_needle_position(self, speed):
        # Define the speed limits and angle range
        min_speed = 0  # Minimum speed
        max_speed = 1000  # Maximum speed (adjust this value accordingly)
        min_angle = 120  # Minimum angle for the needle
        max_angle = -280   # Maximum angle for the needle

        # Calculate the angle based on the speed value
        angle = min_angle + (max_angle - min_angle) * ((speed - min_speed) / (max_speed - min_speed))

        # Update the needle position
        x = self.width / 2
        y = self.height / 2
        angle_rad = math.radians(angle)
        length = min(self.width, self.height) / 2.2
        x2 = x + math.sin(angle_rad) * length
        y2 = y - math.cos(angle_rad) * length
        self.coords(self.needle, x, y, x2, y2)

        # Update the speed label text
        self.speed_label.config(text=f"Speed: {speed:.2f}")

root = Tk()
app = Window(root)
root.mainloop()
