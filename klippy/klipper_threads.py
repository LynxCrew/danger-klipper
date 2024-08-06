import sys
import threading
import time


class KlipperThreads:
    def __init__(self):
        self.running = True
        self.printer = None
        self.registered_threads = []

    def init(self, printer):
        self.printer = printer
        return self

    def is_running(self):
        return self.running and (self.printer is None or not self.printer.is_shutdown())

    def register_job(
        self, group=None, target=None, name=None, args=(), kwargs=None, *, daemon=None
    ):
        return KlipperThread(
            self,
            group=group,
            target=target,
            name=name,
            args=args,
            kwargs=kwargs,
            daemon=daemon,
        )

    def end(self):
        self.running = False

    def finalize(self):
        for k_thread in self.registered_threads:
            k_thread.finalize()


class KlipperThread:
    def __init__(
        self,
        k_threads,
        group=None,
        target=None,
        name=None,
        args=(),
        kwargs=None,
        *,
        daemon=None
    ):
        self.k_threads = k_threads
        self.k_threads.registered_threads.append(self)
        self.thread = threading.Thread(
            group=group,
            target=self._run_job,
            name=name,
            args=(target, args),
            kwargs=kwargs,
            daemon=daemon,
        )

    def start(self):
        self.thread.start()

    def _run_job(self, job, args=()):
        wait_time = job(*args)
        while wait_time > 0 and self.k_threads.is_running():
            time.sleep(wait_time)
            wait_time = job(*args)
        self.k_threads.registered_threads.remove(self)
        sys.exit()

    def finalize(self):
        if self.thread is not None and self.thread.is_alive():
            self.thread.join()
