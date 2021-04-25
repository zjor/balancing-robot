import time
from typing import Callable


class TimedTask:
    def __init__(self, delay: float, run: Callable[[float, float], None], start_delay: float = .0):
        self.delay = delay
        self.run = run
        self.start_delay = start_delay
        self.started = False
        self.last_exec_ts = .0

    def loop(self):
        now = time.time()
        dt = now - self.last_exec_ts
        if not self.started:
            if dt >= self.start_delay:
                self.started = True
                self.last_exec_ts = now
        else:
            if dt >= self.delay:
                self.last_exec_ts = now
                self.run(now, dt)

    def run(self, now, dt):
        pass


if __name__ == "__main__":
    def run(now, dt):
        print(f"Called at {now}; dt={dt}")


    tt = TimedTask(delay=0.5, run=run, start_delay=1.5)

    try:
        while True:
            tt.loop()
    except KeyboardInterrupt:
        pass
