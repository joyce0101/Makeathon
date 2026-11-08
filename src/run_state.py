"""Small validated key=value archives for competition progress.

Writes close the temporary file before atomically replacing the archive.
An orphan temporary file is never used for automatic recovery. Flash writes
can block; callers choose safe save points and handle any write exception.
"""

import os
import config


_FIELDS = ("route", "turn_index", "dead_end_idx", "gap_used", "status", "stop_reason", "seq")
_INTEGER_FIELDS = ("route", "turn_index", "dead_end_idx", "gap_used", "seq")
_STATUSES = ("RUNNING", "LOST", "FINISHED")
_REASON_CHARACTERS = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-. "
_MAX_REASON_LENGTH = 64
_MAX_ARCHIVE_LENGTH = 1024


def _paths(path):
    archive = config.RUN_STATE_FILE if path is None else str(path)
    dot = archive.rfind(".")
    slash = archive.rfind("/")
    temporary = archive[:dot] + ".tmp" if dot > slash else archive + ".tmp"
    if temporary == archive:
        temporary = archive + ".tmp"
    return archive, temporary


def _validated(state):
    if not isinstance(state, dict) or len(state) != len(_FIELDS):
        raise ValueError("Run state must contain exactly the required fields")
    for field in _FIELDS:
        if field not in state:
            raise ValueError("Missing run-state field: {}".format(field))
    for field in _INTEGER_FIELDS:
        value = state[field]
        if type(value) is not int:
            raise ValueError("Run-state {} must be an integer".format(field))
    route = state["route"]
    if route < 1 or route > 4 or route not in config.ROUTES:
        raise ValueError("Run-state route must be 1 to 4")
    route_config = config.ROUTES[route]
    if not 0 <= state["turn_index"] <= len(route_config["turns"]):
        raise ValueError("Run-state turn index is outside the route plan")
    if not 0 <= state["dead_end_idx"] <= len(route_config["dead_ends"]):
        raise ValueError("Run-state dead-end index is outside the route queue")
    if state["gap_used"] not in (0, 1):
        raise ValueError("Run-state gap_used must be 0 or 1")
    if state["seq"] < 0:
        raise ValueError("Run-state sequence must be nonnegative")
    if state["status"] not in _STATUSES:
        raise ValueError("Unknown run-state status")
    reason = state["stop_reason"]
    if not isinstance(reason, str) or len(reason) > _MAX_REASON_LENGTH:
        raise ValueError("Run-state stop reason must be short text")
    for character in reason:
        if character not in _REASON_CHARACTERS:
            raise ValueError("Unsafe character in run-state stop reason")
    return {field: state[field] for field in _FIELDS}


def make_state(route, turn_index=0, dead_end_idx=0, gap_used=False,
               status="RUNNING", stop_reason="", seq=0):
    """Build and validate a detached archive record using route-specific bounds."""
    if type(gap_used) is bool:
        gap_used = int(gap_used)
    return _validated({
        "route": route,
        "turn_index": turn_index,
        "dead_end_idx": dead_end_idx,
        "gap_used": gap_used,
        "status": status,
        "stop_reason": stop_reason,
        "seq": seq,
    })


def load(path=None):
    """Return validated progress, or None for missing/unreadable/corrupt data."""
    archive, unused_temporary = _paths(path)
    try:
        with open(archive, "r") as stream:
            text = stream.read(_MAX_ARCHIVE_LENGTH + 1)
        if len(text) > _MAX_ARCHIVE_LENGTH:
            return None
        state = {}
        for line in text.split("\n"):
            if line.endswith("\r"):
                line = line[:-1]
            if not line:
                continue
            if "=" not in line:
                return None
            key, value = line.split("=", 1)
            if key not in _FIELDS or key in state:
                return None
            state[key] = int(value) if key in _INTEGER_FIELDS else value
        return _validated(state)
    except (OSError, ValueError):
        return None


def save(state, path=None):
    """Atomically replace progress and return its incremented sequence copy.

    The input is never mutated. Failed writes or replacements propagate to
    the controller, whose finally block must stop the motors.
    """
    saved = _validated(state)
    archive, temporary = _paths(path)
    previous = load(archive)
    saved["seq"] = max(saved["seq"], previous["seq"] if previous is not None else 0) + 1
    with open(temporary, "w") as stream:
        for field in _FIELDS:
            line = "{}={}\n".format(field, saved[field])
            if stream.write(line) != len(line):
                raise OSError("Incomplete run-state write")
    os.rename(temporary, archive)
    return saved


def delete(path=None):
    """Remove the archive and any orphan temporary file; missing files are safe."""
    archive, temporary = _paths(path)
    for filename in (archive, temporary):
        try:
            os.remove(filename)
        except OSError as error:
            if not error.args or error.args[0] != 2:
                raise
