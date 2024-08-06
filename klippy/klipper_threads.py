import sys
import threading
import time


class KlipperThreads:
    class DummyPrinter:
        def is_shutdown(self):
            return False

    def __init__(self):
        self.running = False
        self.printer = self.DummyPrinter()
        self.registered_threads = []

    def run(self, printer):
        self.printer = True
        self.printer = printer

    def is_running(self):
        return self.running and not self.printer.is_shutdown()

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
        self.thread = threading.Thread(
            group=group,
            target=self._run_job,
            name=name,
            args=(target, args),
            kwargs=kwargs,
            daemon=daemon,
        )
        self.k_threads.registered_threads.append(self)

    def start(self):
        self.thread.start()

    def _run_job(self, job, args=(), **kwargs):
        try:
            wait_time = job(*args, **kwargs)
            while wait_time > 0 and self.k_threads.is_running():
                time.sleep(wait_time)
                wait_time = job(*args, **kwargs)
        finally:
            self.k_threads.registered_threads.remove(self)
            self.thread = None
            sys.exit()

    def finalize(self):
        if self.thread is not None and self.thread.is_alive():
            self.thread.join()
