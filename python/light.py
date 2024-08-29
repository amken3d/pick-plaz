import os
from dotenv import load_dotenv
from typing import List


class Light:
    def __init__(self):
        load_dotenv()
        self.topdn_pixel = int(os.getenv("TOP_LED_PIN"))
        self.botup_pixel = int(os.getenv("BOT_LED_PIN"))
        self.tray_pixel = int(os.getenv("TRAY_LED_PIN"))
        self.topdn_color = list(map(int, os.getenv("TOP_LED_RGB").split(',')))
        self.botup_color = list(map(int, os.getenv("BOT_LED_RGB").split(',')))
        self.tray_color = list(map(int, os.getenv("TRAY_LED_RGB").split(',')))

    def send_rgb_command(self, pixel: int, color: List[int]):
        # TODO: Implement the sending command to the microcontroller via serial or other means
        # The command might look something like "M150 U<Red>G<Green>B<Blue>"
        # Example: rgb_command = f"M150 U{color[0]}G{color[1]}B{color[2]}"
        ...
        pass

    def turn_on(self, pixel: int, color: List[int]):
        self.send_rgb_command(pixel, color)

    def turn_off(self, pixel: int):
        self.send_rgb_command(pixel, [0, 0, 0])

    def control_topdn(self, state: bool):
        if state:
            self.turn_on(self.topdn_pixel, self.topdn_color)
        else:
            self.turn_off(self.topdn_pixel)

    def control_botup(self, state: bool):
        if state:
            self.turn_on(self.botup_pixel, self.botup_color)
        else:
            self.turn_off(self.botup_pixel)

    def control_tray(self, state: bool):
        if state:
            self.turn_on(self.tray_pixel, self.tray_color)
        else:
            self.turn_off(self.tray_pixel)
