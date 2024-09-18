import _thread
import sys
import threading
import time
from functools import partial
from signal import signal, SIGINT


EXCEPTION = []


def handle_sigint(signalnum, handler):
    raise Exception()


class KlipperThreads:
    def __init__(self):
        self.running = False
        self.registered_threads = []
        signal(SIGINT, handle_sigint)

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
        self.initial_wait_time = None

    def start(self, wait_time=None):
        self.initial_wait_time = wait_time
        self.thread.start()

    def _run_job(self, job, args=(), **kwargs):
        try:
            if self.k_threads.running and self.running and self.initial_wait_time is not None:
                time.sleep(self.initial_wait_time)
                self.initial_wait_time = None
            if self.k_threads.running and self.running:
                wait_time = job(*args, **kwargs)
                while wait_time > 0 and self.k_threads.running and self.running:
                    time.sleep(wait_time)
                    if not self.running:
                        return
                    wait_time = job(*args, **kwargs)
            sys.exit()
        except Exception as e:
            EXCEPTION[0] = e
            _thread.interrupt_main()
        finally:
            self.k_threads.registered_threads.remove(self)
            self.thread = None

    def end(self):
        self.running = False

    def kill(self):
        self.running = False

    def unregister(self):
        self.running = False

    def finalize(self):
        if self.thread is not None and self.thread.is_alive():
            self.thread.join()
