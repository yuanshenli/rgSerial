from threading import Thread, Lock, Event
from rg_serial import *
from rg_dxl import *
import time


def log(string, color="", style="", back=""):

    color_dict = {
        "k": ";30",
        "r": ";31",
        "g": ";32",
        "y": ";33",
        "b": ";34",
        "p": ";35",
        "c": ";36",
        "w": ";37",
        "": "",
    }
    style_dict = {"N": "0", "B": "1", "I": "3", "U": "4", "": "0"}
    back_dict = {
        "k": ";40",
        "r": ";41",
        "g": ";42",
        "y": ";43",
        "b": ";44",
        "p": ";45",
        "c": ";46",
        "w": ";47",
        "": "",
    }
    ENDC = "\033[0m"
    cur_color = "\033[" + style_dict[style] + color_dict[color] + back_dict[back] + "m"
    print(cur_color + str(string) + ENDC)


class ASController:
    def __init__(self, init_action):

        self._n_finger = 3
        self.__motor_cpr = 11836.9 # encoder counter per rel
        self.__com_baud = 1000000
        self.dynamixel_current = [120] * self._n_finger

        self.observation = np.zeros(11)  # obs of all motors
        self.action = init_action
        self.current = np.zeros(self._n_finger)  # current read from dynamixel

        # thread variables (passing among threads)
        self.dxl_position = self.action[: self._n_finger].tolist()  # algorithm -> dxl
        self.dxl_position_read = self.action[:4].tolist()  # dxl -> algorithm
        self.dxl_current_read = [0] * self._n_finger  # dxl -> algorithm
        self.dc_position = [0] * self._n_finger * 2  # algorithm -> dc motor
        self.dc_position_read = [0] * self._n_finger * 2 * 2  # dc motor -> algorithm

        # thread flags
        self.event = Event()  # used for exit
        self.algo_input_flag = Event()
        self.algo_output_dxl_flag = Event()
        self.algo_output_dc_flag = Event()

        # init
        log("Start initializing motors", "y", "B")
        self.dc_motors = SBMotor("/dev/ttyUSB0", self.__com_baud)

        self.init_single_motors(mode=0)

        self.base_motors = RGDynamixel(
            ids=[1, 2, 3],
            init_current=self.dynamixel_current,
            init_pos=self.dxl_position,
            
        )
        self.base_motors.init_motors()

        # thread instances
        self.thread_algo = Thread(target=self.move_to_pos_and_update_obs,)
        self.thread_dxl_io = Thread(target=self.dxl_serial,)
        self.thread_dc_io = Thread(target=self.dc_serial,)
        log("Finish initializing motors", "y", "B")

        # other variables
        self.start_time = [0, 0, 0]
        self.loop_num = [10, 20, 20]
        self.counter = [0, 0, 0]

    def init_single_motors(self, mode=0):
        if mode == 0: # vel control
            self.dc_motors.init_single_motor(4, self.__motor_cpr, 15.0, 0.2, 100.0, ctrl_mode=1)
            self.dc_motors.init_single_motor(5, self.__motor_cpr, 15.0, 0.2, 100.0, ctrl_mode=1)
            self.dc_motors.init_single_motor(6, self.__motor_cpr, 15.0, 0.2, 100.0, ctrl_mode=1)
            self.dc_motors.init_single_motor(7, self.__motor_cpr, 15.0, 0.2, 100.0, ctrl_mode=1)

        elif mode == 1: # pos control
            self.dc_motors.init_single_motor(4, self.__motor_cpr, 14.0, 0.0, 0.3, ctrl_mode=0)
            self.dc_motors.init_single_motor(5, self.__motor_cpr, 14.0, 0.0, 0.3, ctrl_mode=0)
            self.dc_motors.init_single_motor(6, self.__motor_cpr, 14.0, 0.0, 0.3, ctrl_mode=0)
            self.dc_motors.init_single_motor(7, self.__motor_cpr, 14.0, 0.0, 0.3, ctrl_mode=0)


    def check_loop_freq(self, idx, print_freq=False):
        if self.counter[idx] == self.loop_num[idx]:
            self.counter[idx] = idx
            this_time = time.time()
            if print_freq:
                print(
                    "Loop %d freq: %f"
                    % (idx, self.loop_num[idx] / (this_time - self.start_time[idx]))
                )
            self.start_time[idx] = this_time

    def run(self):
        # self.print_block_message('Start communication threads')
        log("Start communication threads", "y", "B")
        self.thread_algo.start()
        self.thread_dxl_io.start()
        self.thread_dc_io.start()

    def stop(self):
        # self.print_block_message('Stop communication threads')
        log("Stop communication threads", "y", "B")
        self.event.set()
        self.thread_algo.join()
        self.thread_dxl_io.join()
        self.thread_dc_io.join()

    def dxl_serial(self):
        while not self.event.is_set():
            # send to dynamixel
            if self.algo_output_dxl_flag.is_set():
                self.algo_output_dxl_flag.clear()
                dxl_pos = self.dxl_position.copy()
                self.base_motors.set_goal_position(dxl_pos)
            # read from dynamixel
            self.base_motors.read_present_position()
            self.base_motors.read_present_current()

            # update shared variables
            self.dxl_position_read = self.base_motors.dxl_present_position.copy()
            self.dxl_current_read = self.base_motors.dxl_present_current.copy()
            self.algo_input_flag.set()
            # print('self.dxl_position_read: ', self.dxl_position_read)
            # check loop frequency
            self.counter[1] += 1
            self.check_loop_freq(1)

    def dc_serial(self):
        while not self.event.is_set():
            # send
            if self.algo_output_dc_flag.is_set():
                self.algo_output_dc_flag.clear()
                dc_pos = self.dc_position.copy()
                self.dc_motors.set_all_velocity(dc_pos)
            # read
            self.dc_motors.request_vals()
            if self.dc_motors.ser.in_waiting:
                sensor_val_full = self.dc_motors.recv_from_serial()
                sensor_val = []
                # for ii in [0, 1, 6, 7]:
                # for ii in [0, 5, 2, 7]:
                for ii in range(8):
                    sensor_val.append(sensor_val_full[ii])
                # print('sensor_val: ', sensor_val)
                # update shared variables
                if sensor_val is not None:
                    self.dc_position_read = sensor_val.copy()
                    self.algo_input_flag.set()

            time.sleep(0.04)

            # check loop frequency
            self.counter[2] += 1
            self.check_loop_freq(2)

    def reset(self, init_action):
        self.action = np.copy(init_action)
        self.observation = np.zeros(11)

    def move_to_pos_and_update_obs(self):
        if "dxl_position_local" not in locals():
            dxl_position_local = self.dxl_position.copy()
        if "dc_position_local" not in locals():
            dc_position_local = self.dc_position.copy()

        while not self.event.is_set():
            # wait until input from motors
            self.algo_input_flag.wait()
            self.algo_input_flag.clear()

            # ------ BEGIN PROCESSING ------ #
            # update obs
            # print('update obs')
            self.current = np.asarray(self.dxl_current_read.copy())
            obs_from_hardware = self.dxl_position_read.copy()
            dc_position_read_local = self.dc_position_read.copy()
            obs_from_hardware.extend(dc_position_read_local)
            self.observation = np.asarray(obs_from_hardware)

            # move to pos
            dxl_position_local = self.action[: self._n_finger].tolist()  # base motor actions
            dc_position_local = self.action[self._n_finger :].tolist()

            # time.sleep(t_delay)
            # ------ END PROCESSING ------ #

            # update shared variables
            self.dxl_position = dxl_position_local.copy()
            self.dc_position = dc_position_local.copy()
            self.algo_output_dxl_flag.set()
            self.algo_output_dc_flag.set()

            # time.sleep(0.1)

            # check loop frequency
            self.counter[0] += 1
            self.check_loop_freq(0)