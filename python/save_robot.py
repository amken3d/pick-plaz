import pick_plaz_robot


class OutOfSaveSpaceException(Exception):
    pass


class SaveRobot(pick_plaz_robot.Robot):
    """ Wrapper for Robot that checks for illegal operations"""

    def __init__(self, comport=None, pos_logger=None):
        super().__init__(comport)

        self.x_bounds = (0, 350)
        self.y_bounds = (0, 350)

        self.pos_logger = pos_logger

    @staticmethod
    def __check_range(x, start_stop):
        """ Return True if the value is outside of the given bounds"""
        if x is None:
            return False
        elif x < start_stop[0] or x > start_stop[1]:
            return True
        else:
            return False

    def drive(self, x=None, y=None, z=None, e=None, a=None, b=None, c=None, f=None, r=None):

        if self.pos_logger is not None:
            if x is not None:
                self.pos_logger["x"] = float(x)
            if y is not None:
                self.pos_logger["y"] = float(y)

        if self.__check_range(x, self.x_bounds):
            raise OutOfSaveSpaceException(
                f"Attempting to drive x={x}, which is outside of save bounds {self.x_bounds}.")

        if self.__check_range(y, self.y_bounds):
            raise OutOfSaveSpaceException(
                f"Attempting to drive y={y}, which is outside of save bounds {self.y_bounds}.")

        return super().drive(x=x, y=y, z=z, e=e, b=b, c=c, f=f, r=r)


def manage_robot(self):
    self.vacuum(False)
    self.valve(False)
    self.drive(z=0)
    self.drive(5, 5)  # drive close to home
    self.done()
    self.dwell(1000)
    self.steppers(False)
    self.light_topdn(False)
    self.light_botup(False)
    self.light_tray(False)
