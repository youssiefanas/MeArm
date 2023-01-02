import tkinter as tk
from geometry_msgs.msg import Twist, Vector3, Pose
import rospy
from std_msgs.msg import Int16
from spatialmath import SE3
from roboticstoolbox import RevoluteDH, DHRobot
import numpy as np
import matplotlib.pyplot as plt

# servo mapping
max_base=120
min_shoulder=70
max_elbow=140
min_gripper=50
min_base=10
max_shoulder=135
min_elbow=100
max_gripper=90
home_base=60
home_shoulder=90
home_elbow=140

# joints limits
j1_max = np.pi/2
j2_max = np.pi*(13/16)
j3_max = -(np.pi/7)-(np.pi/3)

j1_min = -np.pi/2
j2_min = np.pi/7
j3_min = -np.pi*(13/16)

j1_home=0
j2_home=np.pi/2
j3_home=-np.pi/2
home_position_joints=np.array([j1_home,j2_home,j3_home])


class MeArm(DHRobot):
    def __init__(self):
        L0 = RevoluteDH(d=7.5, a=10, alpha=np.pi / 2, qlim=[j1_min, j1_max])
        L1 = RevoluteDH(d=0, a=9, alpha=0, qlim=[j2_min, j2_max])
        L2 = RevoluteDH(d=0, a=9, alpha=0, qlim=[j3_min, j3_max])
        super().__init__([L0, L1, L2], name="MeArm", manufacturer="E-JUST")
        self.home = np.array([0, np.pi / 2, -np.pi / 2])
        self.qz = np.zeros(3)
        self.min = np.array([j1_min, j2_min, j3_min])
        self.max = np.array([j1_max, j2_max, j3_max])
        self.addconfiguration("home", self.home)
        self.addconfiguration("qz", self.qz)


class MeArmGUI:
    def __init__(self):
        self.master = tk.Tk()
        self.master.title("GUI")
        self.master.geometry("900x600")
        self.master.protocol("WM_DELETE_WINDOW", self.on_close_window)
        self.mearm = MeArm()

        # constants
        servo_base_home = self.map_range(
            self.mearm.home[0],
            j1_min,
            j1_max,
            min_base,
            max_base,
        )
        servo_shoulder_home = self.map_range(
            self.mearm.home[1],
            j2_min,
            j2_max,
            min_shoulder,
            max_shoulder,
        )
        servo_elbow_home = self.map_range(
            self.mearm.home[2],
            j3_min,
            j3_max,
            min_elbow,
            max_elbow,
        )
        self.home_position = np.array(
            [
                servo_base_home,
                servo_shoulder_home,
                servo_elbow_home,
            ]
        )

    def init_frames(self):
        frame = tk.Frame(self.master)
        frame.pack()

        # labels
        x_label = tk.Label(frame, text="x")
        y_label = tk.Label(frame, text="y")
        z_label = tk.Label(frame, text="z")

        # create three entry boxes
        self.x_entry = tk.Entry(frame)
        self.y_entry = tk.Entry(frame)
        self.z_entry = tk.Entry(frame)

        # create a button
        calculate_button = tk.Button(frame, text="calculate", command=self.calculate)

        # create clear button to clear the entry boxes and labels
        clear_button = tk.Button(frame, text="clear", command=self.clear)

        # create a slider to control each servo
        self.base_slider = tk.Scale(
            frame,
            from_=min_base,
            to=max_base,
            orient=tk.HORIZONTAL,
            command=self.base_slider_update,
        )
        self.shoulder_slider = tk.Scale(
            frame,
            from_=min_shoulder,
            to=max_shoulder,
            orient=tk.HORIZONTAL,
            command=self.shoulder_slider_update,
        )
        self.elbow_slider = tk.Scale(
            frame,
            from_=min_elbow,
            to=max_elbow,
            orient=tk.HORIZONTAL,
            command=self.elbow_slider_update,
        )

        # create move button to move the arm to the desired position
        gripper_close_button = tk.Button(
            frame, text="gripper close", command=self.gripper_close
        )
        # create button for gripper release
        gripper_release_button = tk.Button(
            frame, text="gripper release", command=self.gripper_release
        )
        # create home button to move the arm to the home position
        home_button = tk.Button(frame, text="home", command=self.home)

        # sliders for the servo
        base_slider_label = tk.Label(frame, text="base")
        shoulder_slider_label = tk.Label(frame, text="shoulder")
        elbow_slider_label = tk.Label(frame, text="elbow")

        # create three labels to display the angles
        self.base_label = tk.Label(frame, text="base = ")
        self.shoulder_label = tk.Label(frame, text="shoulder = ")
        self.elbow_label = tk.Label(frame, text="elbow = ")

        # create three labels to display the servo angles
        self.servo_base_label = tk.Label(frame, text="servo_base = ")
        self.servo_shoulder_label = tk.Label(frame, text="servo_shoulder = ")
        self.servo_elbow_label = tk.Label(frame, text="servo_elbow = ")

        # general error message
        self.error_label = tk.Label(frame, text="error")

        # place the labels and entry boxes
        x_label.grid(row=0, column=0)
        y_label.grid(row=1, column=0)
        z_label.grid(row=2, column=0)
        self.x_entry.grid(row=0, column=1)
        self.y_entry.grid(row=1, column=1)
        self.z_entry.grid(row=2, column=1)
        calculate_button.grid(row=3, column=1)
        clear_button.grid(row=3, column=2)
        # -----------------
        home_button.grid(row=3, column=3)
        gripper_close_button.grid(row=3, column=4)
        gripper_release_button.grid(row=3, column=5)
        # move_button.grid(row = 3, column = 6)

        # sliders for servos
        self.base_slider.grid(row=4, column=1)
        self.shoulder_slider.grid(row=5, column=1)
        self.elbow_slider.grid(row=6, column=1)

        self.base_label.grid(row=4, column=0)
        self.shoulder_label.grid(row=5, column=0)
        self.elbow_label.grid(row=6, column=0)
        self.servo_base_label.grid(row=7, column=0)
        self.servo_shoulder_label.grid(row=8, column=0)
        self.servo_elbow_label.grid(row=9, column=0)
        self.error_label.grid(row=10, column=0)

    def home(self):
        self.mearm.plot(home_position_joints, dt=1)
        self.servo_readings = Vector3()
        self.servo_readings.x = self.home_position[0]
        self.servo_readings.y = self.home_position[1]
        self.servo_readings.z = self.home_position[2]

        # publish the servo_base, servo_shoulder
        # servo_elbow to the topic with msg type Vector3
        self.servo_joints_pub.publish(self.servo_readings)

    def __call__(self) -> None:
        self.servo_node_init()
        self.init_frames()
        self.master.mainloop()

    def servo_node_init(self):
        rospy.init_node("servo_node", anonymous=True)
        self.servo_readings = Vector3()
        # servo_readings.x = servo_base
        # servo_readings.y = servo_shoulder
        # servo_readings.z = servo_elbow
        # publish the servo_base, servo_shoulder, servo_elbow to the topic with msg type Vector3
        self.servo_joints_pub = rospy.Publisher("/servo_joints", Vector3, queue_size=10)
        self.servo_gripper = rospy.Publisher("/servo_gripper", Int16, queue_size=10)

    def gripper_close(self):
        servo_gripper_msg = Int16()
        servo_gripper_msg.data = 50
        # publish the seevo_gripper to the topic
        servo_gripper = rospy.Publisher("/servo_gripper", Int16, queue_size=10)
        servo_gripper.publish(servo_gripper_msg.data)

    def gripper_release(self):
        servo_gripper_msg = Int16()
        servo_gripper_msg.data = 90
        # publish the seevo_gripper to the topic
        servo_gripper = rospy.Publisher("/servo_gripper", Int16, queue_size=10)
        servo_gripper.publish(servo_gripper_msg.data)

    def base_slider_update(self, val):
        servo_base = val
        self.servo_base_label.config(text="servo_base = " + str(servo_base))
        self.servo_joints_pub.publish(self.servo_readings)
        self.mover_slider()

    def shoulder_slider_update(self, val):
        servo_shoulder = val
        self.servo_shoulder_label.config(text="servo_shoulder = " + str(servo_shoulder))
        self.servo_joints_pub.publish(self.servo_readings)
        self.mover_slider()

    def elbow_slider_update(self, val):
        servo_elbow = val
        self.servo_elbow_label.config(text="servo_elbow = " + str(servo_elbow))
        self.servo_joints_pub.publish(self.servo_readings)
        self.mover_slider()

    def mover_slider(self):
        servo_base = self.base_slider.get()
        servo_shoulder = self.shoulder_slider.get()
        servo_elbow = self.elbow_slider.get()
        self.servo_readings = Vector3()
        self.servo_readings.x = servo_base
        self.servo_readings.y = servo_shoulder
        self.servo_readings.z = servo_elbow
        # publish the servo_base, servo_shoulder, servo_elbow to the topic with msg type Vector3
        self.servo_joints_pub.publish(self.servo_readings)

    def clear(self):
        self.x_entry.delete(0, "end")
        self.y_entry.delete(0, "end")
        self.z_entry.delete(0, "end")
        self.error_label.config(text="error message: ")
        self.base_label.config(text="servo_base = ")
        self.shoulder_label.config(text="servo_shoulder =")
        self.elbow_label.config(text="servo_elbow = ")
        self.servo_base_label.config(text="servo_base = ")
        self.servo_shoulder_label.config(text="servo_shoulder =")
        self.servo_elbow_label.config(text="servo_elbow = ")

    def calculate(self):
        x = float(self.x_entry.get())
        y = float(self.y_entry.get())
        z = float(self.z_entry.get())
        # make sure that the values are in the range
        # it is not allowed to have a value out of the range
        self.error_label.config(text="error message: ")
        while x < 0 or x > 24:
            self.error_label.config(text="error: x is out of range")
            # clear the entry
            # enter again
            return

        while y < -24 or y > 24:
            self.error_label.config(text="error: y is out of range")
            return
        while z < 2 or z > 21:
            self.error_label.config(text="error: z is out of range")
            return

        g = SE3(x, y, z)
        print(g)

        # make the inverse kinematics of the robot using ik_lm_chan function
        # s=meArm.ik_lm_chan(g, q0=meArm.home, ilimit=1000, tol=1e-6, we=[1, 1, 1, 0, 0, 0])
        # joints=s[0]
        s = self.mearm.ikine_LM(g, q0=self.mearm.home)
        joints = s.q
        print(joints)
        # meArm.plot(joints,dt=1)
        self.mearm.plot(joints, dt=1)
        base_joint = joints[0]
        shoulder_joint = joints[1]
        elbow_joint = joints[2]
        # check the range of the joints
        print(joints)
        while base_joint < j1_min or base_joint > j1_max:
            print("sq0: ", base_joint)
            print("j1_min = ", j1_min, "j1_max = ", j1_max)
            self.error_label.config(text="error: base is out of range")
            return
        while shoulder_joint < j2_min or shoulder_joint > j2_max:
            print("sq1: ", shoulder_joint)
            print("j2_min = ", j2_min, "j2_max = ", j2_max)
            self.error_label.config(text="error: shoulder is out of range")
            return
        while elbow_joint < j3_min or elbow_joint > j3_max:
            self.error_label.config(text="error: elbow is out of range")
            return

        self.base_label.config(text="base = " + str(base_joint))
        self.shoulder_label.config(text="shoulder = " + str(shoulder_joint))
        self.elbow_label.config(text="elbow = " + str(elbow_joint))

        servo_base = self.map_range(base_joint, j1_min, j1_max, min_base, max_base)
        # check the range of the servo_base
        # while servo_base > min_base or servo_base < max_base:
        #     print(servo_base)
        #     self.error_label.config(text="error: servo_base is out of range")
        #     return
        if servo_base > max_base:
            servo_base = max_base
        if servo_base < min_base:
            servo_base = min_base
        servo_shoulder = self.map_range(shoulder_joint, j2_min, j2_max,min_shoulder,max_shoulder)
        # while servo_shoulder < min_shoulder or servo_shoulder > max_shoulder:
        #     self.error_label.config(text="error: servo_shoulder is out of range")
        #     return
        if servo_shoulder > max_shoulder:
            servo_shoulder = max_shoulder
        if servo_shoulder < min_shoulder:
            servo_shoulder = min_shoulder

        servo_elbow = self.map_range(elbow_joint, j3_min, j3_max, min_elbow, max_elbow)
        # while servo_elbow > min_elbow or servo_elbow < max_elbow:
        #     self.error_label.config(text="error: servo_elbow is out of range")
        #     return
        if servo_elbow > max_elbow:
            servo_elbow = max_elbow
        if servo_elbow < min_elbow:
            servo_elbow = min_elbow

        self.servo_base_label.config(text="servo_base = " + str(servo_base))
        self.servo_shoulder_label.config(text="servo_shoulder = " + str(servo_shoulder))
        self.servo_elbow_label.config(text="servo_elbow = " + str(servo_elbow))
        self.servo_readings.x=int(servo_base)
        self.servo_readings.y=int(servo_shoulder)
        self.servo_readings.z=int(servo_elbow)
        self.servo_joints_pub.publish(self.servo_readings)

    def on_close_window(self):
        plt.close()
        self.master.destroy()

    @staticmethod
    def map_range(x, old_min, old_max, new_min, new_max):
        """_summary_

        Args:
            x (_type_): _description_
            old_min (_type_): _description_
            old_max (_type_): _description_
            new_min (_type_): _description_
            new_max (_type_): _description_

        Returns:
            _type_: _description_
        """
        old_range = old_max - old_min
        new_range = new_max - new_min
        return (((x - old_min) * new_range) / old_range) + new_min


if __name__ == "__main__":
    gui = MeArmGUI()
    gui()