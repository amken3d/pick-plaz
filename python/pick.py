
import time
import json

import numpy as np
import cv2

import calibrator

import debug

AUTO_DETECT_ZONE_MARGIN = 4

class NoPartFoundException(Exception):
    pass

class Picker():

    def __init__(self, cal):

        self.min_area_mm2 = 0.75

        self.res = 20
        self.cam_range = 20

        self.DX = 70.75
        self.DY = -17.59

        try:
            with open("picker.json", "r") as f:
                d = json.load(f)
            self.DX = d["DX"]
            self.DY = d["DY"]
        except FileNotFoundError:
            pass

        self.PICK_Z = -18

        h = calibrator.Homography(cal, self.res, (int(self.res*self.cam_range),int(self.res*self.cam_range)))
        self.ip = calibrator.ImageProjector(h, border_value=(0,0,0))

    def find_part_from_feeder(self, feeder, robot, camera):

        if feeder["type"] == 0:
            return self._find_in_tray(feeder, robot, camera)
        else:
            raise ValueError(f"feeder type {feeder['type']} pick is not implemented'")

    def _find_in_tray(self, feeder, robot, camera):

        r = self.cam_range + AUTO_DETECT_ZONE_MARGIN

        w = feeder["width"] - r
        h = feeder["height"] - r
        xs = feeder["x"] + r/2 + np.linspace(0, w, 2 + int(np.floor(w/(r*2/3))))
        ys = feeder["y"] + r/2 + np.linspace(0, h, 2 + int(np.floor(h/(r*2/3))))
        tray_angle = feeder["rot"] #FIXME not used yet
        search_positions = np.stack(np.meshgrid(xs, ys), axis=-1).reshape((-1,2))

        # self._plot_search_positions(search_positions, feeder)

        robot.light_topdn(False)

        if "last_found_index" in feeder:
            last_found_index = feeder["last_found_index"]
            search_positions = np.roll(search_positions, -last_found_index, axis=0)
        else:
            last_found_index = 0

        for robot_pos in search_positions:

            robot.drive(*robot_pos)
            robot.done()
            time.sleep(0.5)
            image = camera.cache["image"]
            image = self.ip.project(image)

            p, a = self._find_components(image)
            if len(p):
                break
            last_found_index = last_found_index + 1 % len(search_positions)
        else:
            last_found_index = 0
            raise NoPartFoundException("Could not find part to pick")

        feeder["last_found_index"] = last_found_index

        pos = np.array(p[0])
        pos = tuple((pos / self.res) - (self.cam_range/2) + robot_pos)

        #drive to part and measure again without paralax
        robot_pos = pos
        robot.drive(*pos)
        robot.done()
        time.sleep(0.5)
        image = camera.cache["image"]
        image = self.ip.project(image)

        p, a = self._find_components(image)
        if len(p):
            pos = np.array(p[0])
            pos = tuple((pos / self.res) - (self.cam_range/2) + robot_pos)
        else:
            raise NoPartFoundException("Could not find part to pick")

        robot.light_topdn(True)

        #angle in degrees
        return (pos[0], pos[1], a[0])

    def detect_pick_location(self, robot_pos, robot, camera):

        robot.light_topdn(False)
        robot.drive(*robot_pos)
        robot.done()
        time.sleep(0.5)
        image = camera.cache["image"]
        robot.light_topdn(True)
        image = self.ip.project(image)

        p, a = self._find_components(image)
        if len(p):
            pos = np.array(p[0])
            pos = tuple((pos / self.res) - (self.cam_range/2) + robot_pos)
        else:
            raise NoPartFoundException("Could not find part to pick")

        return pos[0], pos[1], a[0]

    def pick(self, robot, x, y, a):
        robot.vacuum(True)
        robot.valve(False)
        robot.drive(x=x+self.DX, y=y+self.DY)
        robot.drive(e=a, f=200)
        robot.drive(z=self.PICK_Z)
        robot.done()
        robot.valve(True)
        robot.drive(z=0)
        robot.drive(e=0, f=200)

    def place(self, robot, x, y, a):
        robot.drive(x=x+self.DX, y=y+self.DY)
        robot.drive(e=a, f=200)
        robot.drive(z=self.PICK_Z)
        robot.done()
        robot.valve(False)
        robot.vacuum(False)
        robot.drive(z=0)

    def calibrate_legacy(self, pos, robot, camera):

        x1, y1, a = self.detect_pick_location(pos, robot, camera)
        self.pick(robot, x1, y1, 0)
        self.place(robot, x1, y1, 180)

        x2, y2, a = self.detect_pick_location((x1, y1), robot, camera)
        self.pick(robot, x1, y1, 0)
        self.place(robot, x1, y1, 180)

        robot.drive(x1, y1)

        correction_x = (x2-x1)/2
        correction_y = (y2-y1)/2
        self.DX -= correction_x
        self.DY -= correction_y

        d = {
            "DX" : self.DX,
            "DY" : self.DY,
        }
        with open("picker.json", "w") as f:
            json.dump(d, f)

        print(f"Picker calibration correction : x={correction_x:.3f}, y={correction_y:.3f}")

    def calibrate(self, pos, robot, camera):

        x, y = pos
        positions = []
        x0, y0, _ = self.detect_pick_location((x, y), robot, camera)
        for _ in range(6):
            self.pick(robot, x0, y0, 0)
            self.place(robot, x0, y0, 360/6)
            x, y, _ = self.detect_pick_location((x, y), robot, camera)
            positions.append((x, y))

        positions = np.asarray(positions)
        a, b, r = taubin(positions)

        distances = np.linalg.norm(positions - [a, b], axis=-1)
        rms_error = np.sqrt(np.mean((distances - r)**2))

        correction_x = a - x0
        correction_y = b - y0
        self.DX -= correction_x
        self.DY -= correction_y

        d = {
            "DX" : self.DX,
            "DY" : self.DY,
        }
        with open("picker.json", "w") as f:
            json.dump(d, f)

        print(f"Picker calibration correction : x={correction_x:.3f}, y={correction_y:.3f}, rms_error={rms_error:.3f}")

    def _find_components(self, image, plot=False):

        from skimage.measure import label, regionprops
        import math

        blur = cv2.GaussianBlur(image,(11,11),0)
        threshold, binary = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        projected = binary



        disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        projected = cv2.morphologyEx(projected, cv2.MORPH_OPEN, disk)

        #floodfill filter
        shape = (binary.shape[0]+2, binary.shape[1]+2)
        outer = np.ones(shape, dtype=np.uint8)*255
        inner = outer[1:-1, 1:-1]
        inner[:] = binary
        cv2.floodFill(outer, None, (0,0), 0)
        projected = inner

        debug.set_image("PickDetection", projected)

        if plot > 1:
            plt.imshow(projected)
            plt.show()

        # %% Find positions/rotations
        image = projected

        if plot > 1:
            fig, ax = plt.subplots()
            ax.imshow(image, cmap=plt.cm.gray)

        image = image.astype(np.float32) / 255

        l = label(image)
        regions = regionprops(l)

        positions = []
        angles = []

        for props in regions:

            if props.area < self.min_area_mm2 * (self.res ** 2):
                continue

            y0, x0 = props.centroid

            positions.append((x0, y0))

            orientation = props.orientation
            angle = (orientation*180/math.pi)
            angle = angle % 90
            if angle > 45: angle -= 90
            angles.append(angle)

            x1 = x0 + math.cos(orientation) * 0.5 * props.minor_axis_length
            y1 = y0 - math.sin(orientation) * 0.5 * props.minor_axis_length
            x2 = x0 - math.sin(orientation) * 0.5 * props.major_axis_length
            y2 = y0 - math.cos(orientation) * 0.5 * props.major_axis_length

            if plot > 1:
                ax.plot((x0, x1), (y0, y1), '-r', linewidth=2.5)
                ax.plot((x0, x2), (y0, y2), '-r', linewidth=2.5)
                ax.plot(x0, y0, '.g', markersize=15)

                minr, minc, maxr, maxc = props.bbox
                bx = (minc, maxc, maxc, minc, minc)
                by = (minr, minr, maxr, maxr, minr)
                ax.plot(bx, by, '-b', linewidth=2.5)

        if plot > 1:
            plt.show()

        positions = np.array(positions)
        angles = np.array(angles)

        return positions, angles


    def make_collage(self, robot, camera):
        # TODO move this somewhere else

        x0, y0 = 103, 125

        mm_step = self.ip.homography.size_mm[0]
        pix_step = self.ip.homography.size_pix[0]

        s = int(10 * pix_step)

        res = np.empty((s, s), np.uint8)

        for x in range(10):
            r = range(10)

            if x % 2:
                r = reversed(r)

            for y in r:
                robot.drive(x0+x*mm_step, y0+y*mm_step)
                robot.done()
                time.sleep(0.5)

                image = camera.cache["image"]
                image = self.ip.project(image)
                res[y*pix_step:(y+1)*pix_step, x*pix_step:(x+1)*pix_step] = image

        cv2.imwrite(f"collage.jpg", res)

    def _plot_search_positions(self, search_positions, feeder):
        import matplotlib.pyplot as plt

        r = self.cam_range/2
        for x, y in search_positions:
            plt.plot([x-r, x-r, x+r, x+r, x-r],[y-r, y+r, y+r, y-r, y-r], "-")
            plt.plot([x],[y], "o")

        x, y, w, h = feeder["x"], feeder["y"], feeder["width"], feeder["height"]

        plt.plot([x, x, x+w, x+w, x], [y, y+h, y+h, y, y], "o-")
        plt.axis("equal")
        plt.savefig("plot.png")
        plt.close()

def picker_test():
    import pickle

    with open("cal.pkl", "rb") as f:
        cal = pickle.load(f)

    p = Picker(cal)

    import json
    with open("web/api/data.json", "r") as f:
        data = json.load(f)

    p.find_part_from_feeder(data["feeder"]["tray 0"], None, None)

    print("done")

def taubin(p):
    """
    Circle fit by Taubin
    p : array[number of samples, 2]

    returns (x, y, r) coordinates and radius
    """
    p = np.asarray(p)
    X = p[:,0] - np.mean(p[:,0])
    Y = p[:,1] - np.mean(p[:,1])

    Z = X * X + Y * Y
    Zmean = np.mean(Z)
    Z0 = (Z - Zmean) / (2 * np.sqrt(Zmean))
    ZXY = np.array([Z0, X, Y]).T
    U, S, V = np.linalg.svd(ZXY, full_matrices=False)
    A0, A1, A2 = V[2]

    A0 /= np.sqrt(Zmean)

    x = -A1 / (2*A0) + np.mean(p[:,0])
    y = -A2 / (2*A0) + np.mean(p[:,1])
    r = np.abs(np.sqrt(A1*A1 + A2*A2 + A0*A0*Zmean) / A0)

    return x, y, r

if __name__ == "__main__":
    picker_test()
