import logging
import time
import queue
import pickle
import os
from sys import exc_info

import cv2
import numpy as np

import camera
import save_robot
import bottle_svr
import fiducial
import calibrator
import pick
import camera_cal
import data_manager
import belt
import tray
import roll
import eye
import json
import config_old
import toml
import PCB
import light
from dotenv import load_dotenv
from hole_finder import NoBeltHoleFoundException
from hole_finder import HoleFinder



class LiveCam:

    def __init__(self, camera, cal, nav_camera):
        self.camera = camera
        res = nav_camera["res"]
        width = nav_camera["width"]
        height = nav_camera["height"]
        h = calibrator.Homography(cal, int(res), (int(res * width), int(res * height)))
        self.ip = calibrator.ImageProjector(h, border_value=(31, 23, 21))

    def get_cam(self):

        cam_image = self.camera.cache["image"]
        cam_image = cv2.cvtColor(cam_image, cv2.COLOR_GRAY2BGR)
        if self.ip is not None:
            cam_image = self.ip.project(cam_image)
        else:
            return cam_image


class AbortException(Exception):
    """ All calibration is broken"""
    pass


class StateContext:
    def __init__(self, robot, camera, context, context_manager, event_queue, light):

        logging.debug("Initializing machine parameters. Please ensure you have created the config.toml file.")
        config = toml.load("config.toml")
        print(config)
        self.light = light
        self.robot = robot
        self.camera = camera
        self.event_queue = event_queue
        self.context = context
        self.context_manager = context_manager
        self.testplacepos = 0

        self.alert_id = 0
        self.do_pause = False

        logging.debug("Initializing navigation parameters.")
        self.nav = {
            "camera": config["camera"],
            "bed": config["machine"],
            "bed_shapes": config["work_area"],
            "pcb": {
                "transform": [1, 0, 0, -1, 10, -10],
                "transform_mse": 0.1,
                "fiducials": {},
            },
            # "pcb": {
            #     "transform": [0.7731953990297232, 0.6349855425247388, 0.6334672579790972, -0.7721192569425455, 58.60784553169759, 195.32190030743706],
            #     "transform_mse": 2.271919407036514e-26,
            #     "fiducials": {
            #         "V1": [76.68486786123627, 191.42850482080448],
            #         "V2": [62.53430535086904, 213.40856212888522],
            #         "V3": [40.5308232021589, 199.21529579406945]
            #     }
            # },
            "detection": {
                "fiducial": [0, 0],
                "belt": [0, 0]
            },
            "state": "idle",
            "light": {
                "topdn": True,
                "botup": False,
                "tray": False
            }
        }
        logging.info("Initializing robot boundaries.")
        self.robot.x_bounds = (config['machine']['bed_area'][0], config['machine']['bed_area'][2])
        self.robot.y_bounds = (config['machine']['bed_area'][1], config['machine']['bed_area'][3])

        try:
            with open("user/fiducial.json") as f:
                self.nav["pcb"]["fiducials"] = json.load(f)
                self._recalculate_fiducial_transform()
                fiducals_assigned = True
                logging.info("fiducial data restored from 'fiducial.json'")
        except FileNotFoundError:
            logging.warning("Fiducial data file not found; using default values.")
            fiducals_assigned = False

        self.robot.pos_logger = self.nav["camera"]

        #Load camera calibration
        try:
            with open("cal.pkl", "rb") as f:
                self.cal = pickle.load(f)
        except FileNotFoundError:
            logging.error("Calibration file not found; using default calibration.")
            self._push_alert(f"Calibration not found. Using default calibration instead. Please calibrate.")
            with open("default_cal.pkl", "rb") as f:
                self.cal = pickle.load(f)

        wide_eye = eye.Eye(self.robot, self.camera, self.cal, res=20, cam_range=20)
        narrow_eye = eye.Eye(self.robot, self.camera, self.cal, res=60, cam_range=5)

        self.live_cam = LiveCam(self.camera, self.cal, self.nav["camera"])
        self.fd = fiducial.FiducialMultiDetector(narrow_eye)
        self.hole_finder = HoleFinder(narrow_eye)
        self.picker = pick.Picker(wide_eye)
        self.belt = belt.Belt(narrow_eye, self.picker)
        self.tray = tray.Tray(self.picker)
        self.roll = roll.Roll(narrow_eye, self.picker)

        if not fiducals_assigned:
            self.center_pcb()

    def center_pcb(self):
        positions = []
        for part in self.context["bom"]:
            for designator in part["designators"].values():
                if "x" in designator:
                    positions.append((float(designator["x"]), float(designator["y"])))
        positions = np.asarray(positions)
        if positions.size != 0:
            bed_center = [self.nav["bed"][2] / 2, self.nav["bed"][3] / 2]
            x, y = -(np.min(positions, axis=0) + np.max(positions, axis=0)) / 2 + bed_center
            self.nav["pcb"]["transform"] = [1, 0, 0, -1, float(x), float(y)]
        self._reset_fiducials()

    def run(self):

        state = self.init_state
        while (True):
            logging.debug("Transitioning to next state.")
            state = state()

    def idle_state(self):

        logging.info("Entering idle state.")
        self.nav["state"] = "idle"

        item = self.event_queue.get()
        if item["type"] == "init":
            logging.debug("Received 'init' event, transitioning to init state.")
            return self.init_state
        else:
            logging.debug(f"Received event: {item['type']}, handling as common event.")
            self._handle_common_event(item)

        return self.idle_state

    def init_state(self):
        """
        Initializes the machine to its starting configuration.
        """
        logging.info("Entering init state.")
        self.nav["state"] = "init"

        #set robot to initial state
        try:
            logging.debug("Resetting machine to initial state.")
            self.robot.vacuum(False)
            self.robot.valve(False)
            self.robot.home()
            logging.info("Machine initialized successfully.")
        except AbortException as e:
            logging.error("Initialization aborted.", exc_info=True)
            self._push_alert(e)
            return self.idle_state

        return self.setup_state

    def get_cam(self):
        """
        Get live camera image.
        """
        logging.debug("Fetching live camera image.")
        return self.live_cam.get_cam()

    def _pcb2robot(self, x, y):
        """
                Transforms PCB coordinates to robot coordinates.

                Args:
                    x (float): X coordinate on the PCB.
                    y (float): Y coordinate on the PCB.

                Returns:
                    tuple: Transformed (x, y) coordinates for the robot.
        """
        logging.debug(f"Transforming PCB coordinates to robot coordinates: ({x}, {y})")
        m = np.array(self.nav["pcb"]["transform"]).reshape((3, 2)).T
        x, y = m[:2, :2] @ (x, y) + m[:2, 2]
        return x, y

    def _pcb2robot2(self, x, y, a):
        m = np.array(self.nav["pcb"]["transform"]).reshape((3, 2)).T
        a = a * np.pi / 180
        logging.debug(f"Converted angle to radians: ")
        x2 = x + np.cos(a)
        y2 = y + np.sin(a)

        x, y = m[:2, :2] @ (x, y) + m[:2, 2]

        x2, y2 = m[:2, :2] @ (x2, y2) + m[:2, 2]

        a = np.arctan2(y2 - y, x2 - x)
        a = a * 180 / np.pi

        return x, y, a

    def setup_state(self):
        """ In this state, the user makes machine setup and can freely roam the pick-plaz bed"""
        logging.info("Entering setup state.")
        self.nav["state"] = "setup"
        self._prepare_light()

        try:
            item = self.event_queue.get()
            logging.debug(f"Event received: {item}")
            if item["type"] == "setpos":
                logging.info("Handling 'setpos' event.")
                x = item["x"]
                y = item["y"]
                if item["system"] == "pcb":
                    x, y = self._pcb2robot(x, y)
                if item["system"] == "relative":
                    x = self.nav["camera"]["x"] + x
                    y = self.nav["camera"]["y"] + y

                self.robot.default_settings()
                self.robot.drive(x, y)
                logging.debug(f"Robot driven to position: ({x}, {y}).")
                # p.make_collage(self.robot, self.camera)

                if self.event_queue.empty():
                    #only do the image work if no other command is pending.
                    #this makes it move quicker between waypoints and navigating.
                    #there is still a big lag spike everytime this section is attempted
                    try:
                        self.nav["detection"]["belt"] = self.hole_finder.find_hole()
                        logging.info("Belt hole detected.")
                    except NoBeltHoleFoundException:
                        logging.warning("No belt hole found.")
                        pass
                    if self.event_queue.empty():
                        #try to abort if possible, else make the next visual search
                        try:
                            self.nav["detection"]["fiducial"] = self.fd()
                            logging.info("Fiducial detected.")
                        except fiducial.NoFiducialFoundException:
                            self.nav["detection"]["fiducial"] = (0, 0)
                            logging.warning("No fiducial found; defaulting to (0, 0).")

            elif item["type"] == "event_setfiducial":
                logging.info("Handling 'event_setfiducial' event.")
                if item["method"] == "assign":
                    self.nav["pcb"]["fiducials"][item["id"]] = (item["x"], item["y"])
                    self._recalculate_fiducial_transform()
                    self._save_fiducial_transform()
                    logging.info(f"Assigned fiducial: {item['id']} at position ({item['x']}, {item['y']}).")
                if item["method"] == "unassign":
                    try:
                        del self.nav["pcb"]["fiducials"][item["id"]]
                    except:
                        pass  #just means fiducial was already not assigned
                    self._recalculate_fiducial_transform()
                    self._save_fiducial_transform()
                elif item["method"] == "reset":
                    self._reset_fiducials()
                    self._recalculate_fiducial_transform()
                    self._save_fiducial_transform()
                    logging.info("Reset fiducials to default state.")

            elif item["type"] == "sequence":
                if item["method"] == "play":
                    self.context_manager.file_save()
                    self._reset_error_parts()
                    logging.info("Playing sequence.")
                    return self.run_state
                elif item["method"] == "home":
                    self.robot.home()
                    self.robot.done()
                    self.robot.drive(0, 0)
                    logging.info("Homed robot and driven to origin.")
                elif item["method"] == "motor_off":
                    self.robot.steppers(False)
                    logging.info("Turned off motors.")
                elif item["method"] == "motor_on":
                    self.robot.steppers(True)
                    logging.info("Turned on motors.")
                elif item["method"] == "calibrate_topdn":
                    logging.info("Handling 'calibrate_topdn' event.")
                    self.cal = camera_cal.calibrate(self.robot, self.camera)
                    with open("cal.pkl", "wb") as f:
                        pickle.dump(self.cal, f)
                elif item["method"] == "calibrate_picker":
                    logging.info("Handling 'calibrate_picker' event.")
                    try:
                        pos = (self.nav["camera"]["x"], self.nav["camera"]["y"])
                        self.picker.calibrate(pos, self.robot, self.camera)
                    except Exception as e:
                        logging.error("An error occurred: %s", e, exc_info=True)
                elif item["method"] == "auto_set_fiducial":
                    logging.info("Handling 'auto_set_fiducial' event.")
                    self.robot.light_topdn(True)
                    self.robot.light_tray(False)
                    fiducial_designators = [part["designators"] for part in self.context["bom"] if part["fiducial"]][0]

                    for name, part in fiducial_designators.items():
                        x, y = self._pcb2robot(float(part["x"]), float(part["y"]))

                        self.robot.drive(x, y)
                        self.nav["pcb"]["fiducials"][name] = self.fd()

                elif item["method"] == "shutdown":
                    logging.info("Handling 'auto_set_fiducial' event.")
                    self.robot.light_topdn(False)
                    self.robot.light_botup(False)
                    self.robot.light_tray(False)
                    import subprocess
                    process = subprocess.run(['shutdown', '-h', '+0'],
                                             stdout=subprocess.PIPE,
                                             universal_newlines=True)
                    if process.returncode != 0:
                        logging.error("Shutdown command failed.")
                        self._push_alert("Shutdown failed")

                #Todo: continue logging enhancements

                elif item["method"] == "place_part":
                    name = item["param"]
                    part, partdes = self._get_part_from_designator(name)
                    if part != None:
                        try:
                            self._place_part(part, partdes)
                        except Exception as e:
                            logging.error("An error occurred: %s", e, exc_info=True)
                            self._push_alert(e)
                    else:
                        self._push_alert("designator not found '%s'", name)
                elif item["method"] == "belt_set_start":
                    name = item["param"]
                    self.belt.set_start(self.context["feeder"][name], self.nav["detection"]["belt"])
                elif item["method"] == "belt_set_end":
                    name = item["param"]
                    self.belt.set_end(self.context["feeder"][name], self.nav["detection"]["belt"])
                elif item["method"] == "set_roll_pickpos":
                    name = item["param"]
                    self.roll.set_pickpos(self.context["feeder"][name], self.nav["detection"]["belt"])
                elif item["method"] == "test_feeder":
                    name = item["param"]
                    feeder = self.context["feeder"][name]
                    if feeder["type"] == tray.TYPE_NUMBER:
                        self.tray.pick(feeder, self.robot)
                    elif feeder["type"] == belt.TYPE_NUMBER:
                        self.belt.pick(feeder, self.robot)
                    elif feeder["type"] == roll.TYPE_NUMBER:
                        self.roll.pick(feeder, self.robot)
                    #drive to testplaceposition and place
                    self.testplacepos = (self.testplacepos + 1) % 35
                    self.picker.place(self.robot, 15 + self.testplacepos * 10, 226, 90)
                    self.robot.default_settings()
                elif item["method"] == "view_feeder":
                    name = item["param"]
                    feeder = self.context["feeder"][name]
                    if feeder["type"] == tray.TYPE_NUMBER:
                        self.tray.pick(feeder, self.robot, only_camera=True)
                    elif feeder["type"] == belt.TYPE_NUMBER:
                        self.belt.pick(feeder, self.robot, only_camera=True)
                    elif feeder["type"] == roll.TYPE_NUMBER:
                        self.roll.pick(feeder, self.robot, only_camera=True)
                elif item["method"] == "reset_board":
                    self._reset_for_new_board()
                elif item["method"] == "roll_advance":
                    name = item["param"]
                    feeder = self.context["feeder"][name]
                    if feeder["type"] == roll.TYPE_NUMBER:
                        self.roll.advance(feeder, self.robot)
                elif item["method"] == "roll_retract":
                    name = item["param"]
                    feeder = self.context["feeder"][name]
                    if feeder["type"] == roll.TYPE_NUMBER:
                        self.roll.retract(feeder, self.robot)
            elif item["type"] == "light_control":
                channel = item["light"]
                enable = item["state"]
                if channel == "topdn":
                    self.nav["light"]["topdn"] = enable
                elif channel == "botup":
                    self.nav["light"]["botup"] = enable
                elif channel == "tray":
                    self.nav["light"]["tray"] = enable
            else:
                logging.warning(f"Unhandled event type: {item['type']}. Passing to common event handler.")
                self._handle_common_event(item)

        except calibrator.CalibrationError as e:
            self._push_alert(f"Calibration Failed: {e}")
        except save_robot.OutOfSaveSpaceException as e:
            self._push_alert(e)
        except fiducial.NoFiducialFoundException as e:
            self._push_alert(e)
        except pick.NoPartFoundException as e:
            self._push_alert(e)
        except AbortException as e:
            self._push_alert(e)
        except Exception as e:
            logging.error("An error occurred: %s", e, exc_info=True)
            self._push_alert(e)
            return self.idle_state

        return self.setup_state

    def run_state(self):
        """ Parts of the BOM are being pick-and-placed """

        self.nav["state"] = "run"

        try:
            logging.info("get next part information")
            part, partdes = self._get_next_part_from_bom()
            if part is None:
                self._push_alert("Placing finished")
                return self.setup_state
            self._place_part(part, partdes)

        except pick.NoPartFoundException as e:
            self._push_alert(e)
            return self.setup_state
        except NoBeltHoleFoundException as e:
            self.push_alert(e)
            return self.idle_state
        except save_robot.OutOfSaveSpaceException as e:
            self._push_alert(e)
        except AbortException as e:
            self._push_alert(e)
            return self.idle_state
        if self.do_pause == True:
            #pause was requested.
            return self.idle_state
        return self.run_state

    def _handle_common_event(self, item):
        if item["type"] == "alertquit":
            if "alert" in self.nav:
                del self.nav["alert"]

    def _reset_error_parts(self):
        for part in self.context["bom"]:
            for name, partdes in part["designators"].items():
                if partdes["state"] == data_manager.PART_STATE_ERROR and partdes["place"] and not part["fiducial"]:
                    partdes["state"] = data_manager.PART_STATE_READY

    def _reset_for_new_board(self):
        for part in self.context["bom"]:
            for name, partdes in part["designators"].items():
                if partdes["state"] != data_manager.PART_STATE_SKIP:
                    partdes["state"] = data_manager.PART_STATE_READY

    def _get_part_from_designator(self, name):
        """ find part to place only from its designator """
        for part in self.context["bom"]:
            partdes = part["designators"].get(name)
            if partdes != None:
                return part, partdes
        return None, None

    def _place_part(self, part, partdes):
        self.robot.default_settings()
        partdes["state"] = data_manager.PART_STATE_ERROR

        place_pos = float(partdes["x"]), float(partdes["y"])
        place_angle = float(partdes["rot"]) + float(part["rot"])

        #skip if feeder not defined
        #TODO maybe a bit ugly:
        feeder = part.get("feeder")
        if feeder is None:
            partdes["state"] = data_manager.PART_STATE_ERROR
            return self.run_state
        feeder = self.context["feeder"][feeder]

        self._poll_for_pause()

        logging.info("pick part")
        if feeder["type"] == tray.TYPE_NUMBER:
            self.tray.pick(feeder, self.robot)
        elif feeder["type"] == belt.TYPE_NUMBER:
            self.belt.pick(feeder, self.robot)
        elif feeder["type"] == roll.TYPE_NUMBER:
            self.roll.pick(feeder, self.robot)
        else:
            raise Exception(f"Feeder type {feeder['type']} unknown")

        self._poll_for_pause()

        logging.info("place part")
        x, y = place_pos
        x, y, place_angle = self._pcb2robot2(x, y, place_angle)
        # make sure rotation is minimal
        if place_angle < -180:
            place_angle += 360
        if place_angle > 180:
            place_angle -= 360
        # angle is in pcb coordinates, thus inverted
        self.picker.place(self.robot, x, y, -place_angle)

        logging.info("update part state")
        partdes["state"] = data_manager.PART_STATE_PLACED

        self.robot.default_settings()
        self._poll_for_pause()

    def _poll_for_pause(self):
        try:
            item = self.event_queue.get(block=False)
            if item["type"] == "sequence":
                if item["method"] == "pause":
                    self.do_pause = True
                    return self.setup_state
                if item["method"] == "stop":
                    raise AbortException()
            else:
                self._handle_common_event(item)
        except queue.Empty:
            pass

    def _get_next_part_from_bom(self):
        """ find next part in bom that is eligable for placing"""
        for part in self.context["bom"]:
            if part["place"] == True:
                for name, partdes in part["designators"].items():
                    if "x" not in partdes:
                        continue
                    if partdes["state"] == data_manager.PART_STATE_READY and partdes["place"] and not part["fiducial"]:
                        return part, partdes
        return None, None

    def _push_alert(self, msg, answers=None):

        if answers is None:
            answers = ["OK"]

        self.nav["alert"] = {
            "id": self.alert_id,
            "msg": str(msg),
            "answers": [str(a) for a in answers],
        }

        self.alert_id += 1

    def _reset_fiducials(self):
        self.nav["pcb"] = {
            "transform": [1, 0, 0, -1, 10, -10],
            "transform_mse": 0.1,
            "fiducials": {},
        }

    def _recalculate_fiducial_transform(self):
        try:
            # Refreshes everything on pcb except the fiducial
            fiducial_designators_list = [part["designators"] for part in self.context["bom"] if part["fiducial"]]
            if fiducial_designators_list:
                fiducial_designators = fiducial_designators_list[0]
                transform, mse = fiducial.get_transform(self.nav["pcb"]["fiducials"], fiducial_designators)
                self.nav["pcb"]["transform"] = transform
                self.nav["pcb"]["transform_mse"] = float(mse)
            else:
                logging.error("No fiducial designators found in BOM")
                logging.info("No fiducial designators found in BOM")
        except Exception as e:
            logging.error("An error occurred: %s", e, exc_info=True)
            print(e)
            pass

    def _save_fiducial_transform(self):
        #saves fiducial data to user files
        with open("user/fiducial.json", "w") as f:
            json.dump(self.nav["pcb"]["fiducials"], f)
            print("fiducial data saved to 'fiducial.json'")

    #set all lamps to normal operating mode
    def _prepare_light(self):
        light = self.nav["light"]
        self.robot.light_topdn(light["topdn"])
        self.robot.light_tray(light["tray"])
        #botup is missing, but it's not working or used


def createdir(directory):
    try:
        os.makedirs(directory)
    except:
        pass


# Introduce constants for literals
DIRECTORY1 = "user/context"
DIRECTORY2 = "template"
START_MESSAGE = "pick-plaz starting..."
SIMULATION_MESSAGE = "starting in simulation mode"
CONNECTED_MESSAGE = "Connected to controller on serial port"
CAMERA_MESSAGE = "Camera connected"
RUNNING_MESSAGE = "pick-plaz running. press [ctrl]+[C] to shutdown"
PARKING_MESSAGE = "Parking the machine"
FINISH_MESSAGE = "Finished run"


def main(mock=False):
    setup_logging()
    setup_directories()
    queue = create_queue()
    robot = connect_robot(mock)
    camera_thread = connect_camera(mock)
    data_manager = initialize_data()
    state_context = initialize_state_context(robot, camera_thread, data_manager, queue, light)
    server = initialize_server(state_context, data_manager, queue)
    run_application(state_context, camera_thread)
    terminate_application(data_manager, robot)


def setup_logging():
    logging.info(START_MESSAGE)


def setup_directories():
    createdir(DIRECTORY1)
    createdir(DIRECTORY2)


def create_queue():
    return queue.Queue()


def connect_robot(mock):
    if not mock:
        logging.info(CONNECTED_MESSAGE)
    return save_robot.SaveRobot(None if mock else os.getenv("SERIAL_PORT"))


def connect_camera(mock):
    if not mock:
        logging.info(CAMERA_MESSAGE)
        return camera.CameraThread(0)
    else:
        return camera.CameraThreadMock()


def initialize_data():
    return data_manager.ContextManager()


def initialize_state_context(robot, camera_thread, data_manager, queue, light):
    logging.info(
        f"StateContext initialized with: Robot:{robot}, "
        f"Camera Thread:{camera_thread}, "
        f"Data Manager:{data_manager}, "
        f"Queue:{queue}")
    return StateContext(robot, camera_thread, data_manager.get(), data_manager, queue,light)


def initialize_server(state_context, data_manager, queue):
    try:
        server = bottle_svr.BottleServer(
            state_context.get_cam,
            queue.put,
            data_manager,
            state_context.center_pcb,
            state_context.nav
        )
        logging.info("Server initialized successfully.")
        return server
    except Exception as e:
        logging.error(f"Failed to initialize the server: {str(e)}")
        raise


def run_application(state_context, camera_thread):
    try:
        # Assuming camera_thread accommodates context management
        with camera_thread:
            state_context.run()  # Potential Exceptions may originate from here
            logging.info(RUNNING_MESSAGE)

    except KeyboardInterrupt:
        logging.error("The operation was interrupted.")
    except Exception as e:
        logging.error(f"An error occurred: {str(e)}")


def terminate_application(data_manager, robot):
    try:
        logging.info(PARKING_MESSAGE)

        data_manager.file_save()

        robot.manage_robot()

        logging.info(FINISH_MESSAGE)
    except KeyboardInterrupt:
        logging.error("The operation was interrupted.")
    except Exception as e:
        logging.error(f"An error occurred while terminating the application: {str(e)}")
        raise


if __name__ == "__main__":
    main()
