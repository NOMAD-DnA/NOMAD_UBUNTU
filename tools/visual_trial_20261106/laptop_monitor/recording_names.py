"""Safe trial-name filenames for closed GUI recordings. No board access."""
import ctypes
import errno
import os
from pathlib import Path
import re
import unicodedata


def trial_filename_stem(name):
    """Preserve Korean/spaces, sanitize separators, bound encoded filename size."""
    if not isinstance(name, str) or not name.strip():
        raise ValueError("시험 이름을 입력해 주세요.")
    name = unicodedata.normalize("NFC", name.strip())
    name = re.sub(r'[<>:"/\\|?*\x00-\x1f\x7f]', '_', name).strip(' .')
    name = name.encode('utf-8')[:200].decode('utf-8', errors='ignore').rstrip(' .') or '시험'
    if re.fullmatch(r'(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\..*)?', name, re.I):
        name = '_' + name
    return name


def _rename_no_replace(source, destination):
    """Linux atomic no-clobber rename, including shared filesystems.

    Never use exists()+os.rename(): another recording could appear between them.
    Unsupported filesystems raise and preserve the source instead of overwriting.
    """
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, 'renameat2', None)
    if rename is None:
        raise OSError(errno.ENOTSUP, 'This system does not support safe recording rename')
    rename.argtypes = (ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint)
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(source), -100, os.fsencode(destination), 1) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), str(destination))


def rename_trial_recording(path, name):
    """Rename a closed file in place; collisions become name (2).nomad, etc."""
    source = Path(path).expanduser().absolute()
    if source.suffix != '.nomad' or source.is_symlink() or not source.is_file():
        raise ValueError('이름을 바꿀 주행 기록 파일을 확인할 수 없습니다.')
    if any(Path(str(source)+suffix).exists() for suffix in ('-journal', '-wal', '-shm')):
        raise RuntimeError('아직 사용 중인 기록입니다. 기록이 완전히 종료된 뒤 다시 저장해 주세요.')
    stem = trial_filename_stem(name)
    for index in range(1, 10001):
        suffix = '' if index == 1 else ' (%d)' % index
        destination = source.with_name(stem + suffix + '.nomad')
        if source == destination:
            return str(source)
        try:
            _rename_no_replace(source, destination)
            return str(destination)
        except FileExistsError:
            continue
    raise FileExistsError('같은 시험 이름의 기록이 너무 많습니다. 다른 이름을 입력해 주세요.')
