"""Pure timestamp helpers for the laser scan relay."""

NANOSECONDS_PER_SECOND = 1_000_000_000


def scan_age_seconds(now_ns, scan_sec, scan_nanosec):
    """Return the scan age relative to the current clock in seconds."""
    scan_ns = scan_sec * NANOSECONDS_PER_SECOND + scan_nanosec
    return (now_ns - scan_ns) / NANOSECONDS_PER_SECOND


def should_forward_scan(now_ns, scan_sec, scan_nanosec, max_age_s):
    """Return whether the clock has started and the scan is not stale."""
    if now_ns == 0:
        return False
    return scan_age_seconds(now_ns, scan_sec, scan_nanosec) <= max_age_s
