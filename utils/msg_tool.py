import threading


class LatestMsg:

    def __init__(self):
        self.data = None
        self.lock = threading.RLock()
        self.updated: set[threading.Event] = set()

    def put(self, *args):
        with self.lock:
            self.data = args if len(args) > 1 else args[0]
            for event in self.updated: event.set()

    def get(self):
        with self.lock:
            return self.data

    def create_getter(self, timeout=None):
        return self.Getter(self, timeout)

    class Getter:

        def __init__(self,
                     lmsg: "LatestMsg",
                     timeout: float = None):
            self.lmsg = lmsg
            self.timeout = timeout
            self.event = threading.Event()
            with lmsg.lock:
                lmsg.updated.add(self.event)

        def __call__(self):
            if self.event.wait(self.timeout):
                self.event.clear()
                return self.lmsg.get()

        def __del__(self):
            with self.lmsg.lock:
                self.lmsg.updated.remove(self.event)
