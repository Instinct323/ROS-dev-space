import threading


class LatestMsg:

    def __init__(self):
        self.data = None
        self.lock = threading.RLock()
        self.updated: set[threading.Event] = set()

    def put(self, *args):
        with self.lock:
            self.data = args
            for event in self.updated: event.set()

    def get(self):
        with self.lock:
            return self.data

    def create_getter(self, timeout=None):
        with self.lock:
            event = threading.Event()
            self.updated.add(event)

        def get():
            if event.wait(timeout):
                event.clear()
                return self.get()
        return get
