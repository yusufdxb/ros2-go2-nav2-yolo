"""Tests for the scan relay timestamp filter."""

import sys
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from scan_filter import scan_age_seconds, should_forward_scan  # noqa: E402


class ScanFilterTest(unittest.TestCase):
    """Exercise fresh, boundary, stale, and uninitialized clock cases."""

    def test_age_uses_seconds_and_nanoseconds(self):
        """The helper must preserve sub-second timestamp precision."""
        now_ns = 12_500_000_000
        self.assertEqual(scan_age_seconds(now_ns, 12, 250_000_000), 0.25)

    def test_fresh_scan_is_forwarded(self):
        """A scan inside the maximum age remains eligible for relay."""
        self.assertTrue(should_forward_scan(10_000_000_000, 9, 250_000_000, 1.0))

    def test_boundary_scan_is_forwarded(self):
        """A scan exactly at the configured age limit is still valid."""
        self.assertTrue(should_forward_scan(10_000_000_000, 9, 0, 1.0))

    def test_stale_scan_is_dropped(self):
        """A scan one nanosecond beyond the age limit is rejected."""
        self.assertFalse(
            should_forward_scan(10_000_000_000, 8, 999_999_999, 1.0)
        )

    def test_scan_is_dropped_before_sim_clock_starts(self):
        """No scan is relayed while simulated time is still zero."""
        self.assertFalse(should_forward_scan(0, 0, 0, 1.0))


if __name__ == "__main__":
    unittest.main()
