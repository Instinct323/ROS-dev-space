import threading


class LatestMsg:

    def __init__(self):
        self.data = None
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)

    def put(self, *args):
        with self.cond:
            self.data = args
            self.cond.notify()

    def get(self):
        with self.cond:
            if self.data is None: self.cond.wait()
            data, self.data = self.data, None
            return data
