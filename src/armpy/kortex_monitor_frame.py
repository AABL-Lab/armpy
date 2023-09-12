import functools
import message_filters
import tkinter


import kortex_arm


class ListDisplay(tkinter.LabelFrame):
    def __init__(self, parent, label, labels=("x", "y", "z", "r", "p", "y")):
        super().__init__(parent, label)

        self._title = tkinter.Label(self, text=label)
        self._title.grid(row=0, column=0, columnspan=2)

        self._vars = []
        self._labels = []
        self._disp = []
        for i, val in enumerate(labels):
            var = tkinter.StringVar()
            self._vars.append(var)
            label = tkinter.Label(self, text=str(val))
            label.grid(row=i+1, column=0)
            self._labels.append(label)
            disp = tkinter.Label(self, textvariable=var)
            disp.grid(row=i+1, column=1, sticky="E")
            self._disp.append(disp)
            self.rowconfigure(i+1, weight=1)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=4)
    
    def update(self, vals):
        for val, var in zip(vals, self._vars):
            var.set(f"{val:.03f}")

class KortexStatusFrame(tkinter.LabelFrame):
    def __init__(self, parent, arm):
        super().__init__(parent, "Status")

        self._sub = message_filters.Subscriber(*arm.get_feedback_sub_args())
        self._cache = message_filters.Cache(self._sub, cache_size=1, allow_headerless=True)

        self._tool_pose = ListDisplay(self, "Tool pose", ("x", "y", "z", "r", "p", "y"))
        self._tool_pose.grid(row=1, column=0)

        self._tool_twist = ListDisplay(self, "Tool twist", ("vx", "vy", "vz", "dr", "dp", "dy"))
        self._tool_twist.grid(row=1, column=1)

        self._update()

    def _update(self):
        self.update()
        self.after(1000, self._update)

    def update(self):
        # access is threadsafe though not guaranteed
        msg = self._cache.cache_msgs[-1]

        self._tool_pose.update([
            msg.base.tool_pose_x, msg.base.tool_pose_y, msg.base.tool_pose_z,
            msg.base.tool_pose_theta_x, msg.base.tool_pose_theta_y, msg.base.tool_pose_theta_z
        ])
        self._tool_twist.update([
            msg.base.tool_twist_linear_x, msg.base.tool_twist_linear_y, msg.base.tool_twist_linear_z,
            msg.base.tool_twist_angular_x, msg.base.tool_twist_angular_y, msg.base.tool_twist_angular_z
        ])

class KortexMonitorFrame(tkinter.LabelFrame):
    def __init__(self, parent, initial_config):
        super().__init__(parent, "Kortex Monitor")

        self._arm = kortex_arm.Arm()

        self._control_frame = tkinter.Frame(self)
        self._control_frame.grid(row=0, column=0)
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        self._stop = tkinter.Button(self._control_frame, text="Stop", command=self._arm.stop)
        self._stop.grid(row=0, column=0)

        self._clear_faults = tkinter.Button(self._control_frame, text="Clear faults", 
                                            command=functools.partial(self._arm.clear_faults, block=False))
        self._clear_faults.grid(row=0, column=1)

        self._home_arm = tkinter.Button(self._control_frame, text="Home arm", 
                                        command=functools.partial(self._arm.home_arm, block=False))
        self._home_arm.grid(row=1, column=0)

        self._open_gripper = tkinter.Button(self._control_frame, text="Open gripper", 
                                            command=functools.partial(self._arm.open_gripper, block=False))
        self._open_gripper.grid(row=2, column=0)
        self._close_gripper = tkinter.Button(self._control_frame, text="Close gripper", 
                                             command=functools.partial(self._arm.close_gripper, block=False))
        self._close_gripper.grid(row=2, column=1)

        self._estop = tkinter.Button(self._control_frame, background="red", text="ESTOP", command=self._arm.estop)
        self._estop.grid(row=3, column=0, columnspan=2)

        self._status_frame = KortexStatusFrame(self, self._arm)
        self._status_frame.grid(row=0, column=1)
        self.columnconfigure(1, weight=1)

    def get_config(self):
        return dict()
    def set_state(self, state):
        pass

    
