import logging
import sys
import threading
import time


class KlipperThreads:
    def __init__(self):
        self.running = False
        self.registered_threads = []

    def run(self):
        self.running = True

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

    def unregister_job(self, job):
        if job in self.registered_threads:
            job.running = False

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
        self.running = True

    def start(self):
        self.thread.start()

    def _run_job(self, job, args=(), **kwargs):
        try:
            if self.k_threads.running and self.running:
                wait_time = job(*args, **kwargs)
                while wait_time > 0 and self.k_threads.running and self.running:
                    time.sleep(wait_time)
                    if not self.running:
                        return
                    wait_time = job(*args, **kwargs)
        finally:
            self.k_threads.registered_threads.remove(self)
            self.thread = None
            sys.exit()

    def end(self):
        self.running = False

    def kill(self):
        self.running = False

    def unregister(self):
        self.running = False

    def finalize(self):
        if self.thread is not None and self.thread.is_alive():
            self.thread.join()