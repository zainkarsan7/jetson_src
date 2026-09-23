"""Timing diagnostics and a bounded latest-frame worker, independent of ROS."""
from concurrent.futures import ThreadPoolExecutor


def check_age(stamp_ns, now_ns, max_age_s, label='Observation'):
    age = (now_ns-stamp_ns)/1e9
    if age < -0.05:
        raise ValueError(f'{label} timestamp is {-age:.3f}s ahead of ROS time; check clocks/use_sim_time')
    if age > max_age_s:
        raise ValueError(f'{label} is stale: age {age:.3f}s exceeds {max_age_s:.3f}s; '
                         'check processing/delivery delay and clock offset')
    return age


class LatestWorker:
    """One running job and one replaceable waiting item; caller owns all state."""
    def __init__(self, function):
        self.function = function
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='handeye_detector')
        self.waiting = None
        self.running = None
        self.replaced = 0

    def put(self, item):
        if self.waiting is not None:
            self.replaced += 1
        self.waiting = item

    def start(self):
        if self.running is not None or self.waiting is None:
            return False
        item, self.waiting = self.waiting, None
        self.running = (item, self.executor.submit(self.function, item))
        return True

    def take_done(self):
        if self.running is None or not self.running[1].done():
            return None
        item, future = self.running
        self.running = None
        try:
            return item, future.result(), None
        except Exception as exc:
            return item, None, exc

    def close(self):
        self.waiting = None
        self.executor.shutdown(wait=True)
