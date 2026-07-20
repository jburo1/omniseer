import tempfile
import unittest
from pathlib import Path

from omniseer_experiments.system_telemetry import (
    CpuSample,
    SystemTelemetrySampler,
    cpu_percent,
    parse_proc_meminfo,
    parse_proc_stat_cpu,
    read_temperature_c,
)


class SystemTelemetryTests(unittest.TestCase):
    def test_cpu_delta_calculates_busy_percent(self) -> None:
        previous = parse_proc_stat_cpu("cpu  100 0 50 850 0 0 0 0 0 0\n")
        current = parse_proc_stat_cpu("cpu  150 0 70 880 0 0 0 0 0 0\n")

        self.assertAlmostEqual(cpu_percent(previous, current), 70.0)

    def test_cpu_delta_zero_total_falls_back_to_zero(self) -> None:
        sample = CpuSample(idle=10, total=20)

        self.assertEqual(cpu_percent(sample, sample), 0.0)
        self.assertEqual(cpu_percent(None, sample), 0.0)

    def test_meminfo_uses_memavailable(self) -> None:
        sample = parse_proc_meminfo(
            "\n".join(
                [
                    "MemTotal:        8192000 kB",
                    "MemFree:         1024000 kB",
                    "MemAvailable:   6144000 kB",
                    "Buffers:          512000 kB",
                    "Cached:          2048000 kB",
                ]
            )
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.used_mb, 2000.0)
        self.assertEqual(sample.available_mb, 6000.0)

    def test_meminfo_falls_back_without_memavailable(self) -> None:
        sample = parse_proc_meminfo(
            "\n".join(
                [
                    "MemTotal:        4096000 kB",
                    "MemFree:          512000 kB",
                    "Buffers:          256000 kB",
                    "Cached:           512000 kB",
                    "SReclaimable:     256000 kB",
                ]
            )
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.used_mb, 2500.0)
        self.assertEqual(sample.available_mb, 1500.0)

    def test_temperature_reads_millidegrees(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "temp"
            path.write_text("61400\n", encoding="utf-8")

            self.assertEqual(read_temperature_c([path]), 61.4)

    def test_temperature_missing_falls_back_to_none(self) -> None:
        self.assertIsNone(read_temperature_c([Path("/missing/temperature")]))

    def test_sampler_returns_minimum_record_shape(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            proc_stat = root / "stat"
            proc_meminfo = root / "meminfo"
            temperature = root / "temp"
            proc_stat.write_text("cpu  100 0 50 850 0 0 0 0 0 0\n", encoding="utf-8")
            proc_meminfo.write_text("MemTotal: 2048000 kB\nMemAvailable: 1024000 kB\n", encoding="utf-8")
            temperature.write_text("42000\n", encoding="utf-8")
            sampler = SystemTelemetrySampler(
                proc_stat_path=proc_stat,
                proc_meminfo_path=proc_meminfo,
                temperature_paths=[temperature],
                time_ns=lambda: 123,
            )

            record = sampler.sample()

        self.assertEqual(record["schema_version"], 1)
        self.assertEqual(record["source"], "system")
        self.assertEqual(record["recv_ts_ns"], 123)
        self.assertEqual(record["cpu_percent"], 0.0)
        self.assertEqual(record["memory_used_mb"], 1000.0)
        self.assertEqual(record["memory_available_mb"], 1000.0)
        self.assertEqual(record["soc_temp_c"], 42.0)


if __name__ == "__main__":
    unittest.main()
