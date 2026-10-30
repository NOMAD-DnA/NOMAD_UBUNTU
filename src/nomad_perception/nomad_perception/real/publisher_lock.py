"""One real perception publisher per user, domain and resolved output topic."""
import fcntl
import hashlib
import os
from pathlib import Path
import tempfile


class PublisherLock:
    def __init__(self,topic):
        key=hashlib.sha256((os.environ.get('ROS_DOMAIN_ID','0')+topic).encode()).hexdigest()[:20]
        self.file=(Path(tempfile.gettempdir())/f'nomad_real_perception_{os.getuid()}_{key}.lock').open('a+')
        try:fcntl.flock(self.file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            self.file.close();raise RuntimeError('Real perception is already publishing this topic in this ROS domain')

    def close(self):
        self.file.close()
