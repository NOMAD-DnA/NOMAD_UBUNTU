"""Worker lifecycle and atomic state transport, without replay or viewers."""
from pathlib import Path
import json,queue,time

def atomic_json(path,value):
    path=Path(path);tmp=path.with_suffix(path.suffix+".tmp")
    tmp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(",",":")));tmp.replace(path)

class TerrainWorker:
    def status(self):
        path = self.directory / 'status.json'
        return json.loads(path.read_text()) if path.exists() else {'enabled': True, 'saved': False, 'error': None, 'modes': {}}

    def close(self, on_progress=None, stall_timeout_s=90.):
        if self.closed:
            return self.status()
        self.closed = True
        try:
            if self.process.is_alive():
                try:
                    self.queue.put(None, timeout=5)
                except queue.Full as error:
                    self.process.terminate()
                    self.process.join(timeout=5)
                    raise RuntimeError('경사 작업 종료 대기열 시간 초과.') from error
                # Drain pending work while calculation is making progress.
                # Do not kill a worker that is still publishing new results.
                last_update = self.status().get('updated_monotonic')
                last_progress = time.monotonic()
                last_callback = 0.
                while self.process.is_alive():
                    self.process.join(timeout=.5)
                    now = time.monotonic()
                    status = self.status()
                    updated = status.get('updated_monotonic')
                    if updated != last_update:
                        last_update = updated
                        last_progress = now
                    if on_progress is not None and now-last_callback >= 5:
                        on_progress()
                        last_callback = now
                    if now-last_progress > stall_timeout_s:
                        break
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=5)
                raise RuntimeError('경사 작업 종료 제한 시간 초과.')
            state = self.status()
            if self.process.exitcode != 0 or not state.get('saved'):
                raise RuntimeError(state.get('error') or '경사 작업의 정상 종료를 확인하지 못했습니다.')
            return state
        finally:
            self.queue.cancel_join_thread()
            self.queue.close()
