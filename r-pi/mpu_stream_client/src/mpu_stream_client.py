import time
import socket
import pygame as pg
from math import pi, radians, sin, cos
from multiprocessing import Process, Value

WIDTH = 960
HEIGHT = 576


class Colors:
    RED = (200, 80, 80)
    GREEN = (80, 200, 80)
    BLUE = (80, 80, 200)


class Bar:
    def __init__(self, cx, cy, length, angle, color):
        self.cx = cx
        self.cy = cy
        self.length = length
        self.angle = angle
        self.color = color

    def get_left(self):
        return (
            self.cx - (self.length / 2) * cos(radians(self.angle)),
            self.cy + (self.length / 2) * sin(radians(self.angle))
        )

    def get_right(self):
        return (
            self.cx + (self.length / 2) * cos(radians(self.angle)),
            self.cy - (self.length / 2) * sin(radians(self.angle))
        )

    def render(self, surface):
        pg.draw.line(surface, self.color, self.get_left(), self.get_right(), width=2)


def render_column(surface, font, title, cx, angle, color):
    text = font.render(title, True, color)
    text_rect = text.get_rect()
    surface.blit(text, (cx - text_rect.width / 2, 50))
    pitch_bar = Bar(cx, HEIGHT / 2, WIDTH / 4, angle, color)
    pitch_bar.render(window)


def read_mpu_stream(pitch, roll, started):
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client.bind(("", 37020))

    while started.value:
        data, _ = client.recvfrom(1024)
        data = list(map(float, data.split()))
        pitch.value = data[0]
        roll.value = data[1]


if __name__ == "__main__":
    import ctypes

    pg.init()
    pg.display.set_caption("MPU Stream Client")
    window = pg.display.set_mode((WIDTH, HEIGHT))

    font = pg.font.SysFont(None, 24)

    pitch = Value(ctypes.c_double, 0.0)
    roll = Value(ctypes.c_double, 0.0)
    started = Value(ctypes.c_bool, True)

    p = Process(target=read_mpu_stream, args=(pitch, roll, started))
    p.start()

    running = True
    while running:

        window.fill((0, 0, 0))
        render_column(window, font, 'Pitch', WIDTH / 4, pitch.value, Colors.RED)
        render_column(window, font, 'Roll', 3 * WIDTH / 4, roll.value, Colors.GREEN)

        pg.display.flip()

        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                running = False
                started.value = False

    p.join()
