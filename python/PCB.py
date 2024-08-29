from typing import Dict, List


class PCB:
    def __init__(self, transform: List[float], transform_mse: float, fiducials: Dict[str, List[float]]):
        self.transform = transform
        self.transform_mse = transform_mse
        self.fiducials = fiducials


class Detection:
    def __init__(self, fiducial: List[int], belt: List[int]):
        self.fiducial = fiducial
        self.belt = belt
