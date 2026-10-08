import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_scheduler_skips_missed_periods_and_handles_micros_rollover(tmp_path: Path) -> None:
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("g++ is required for the scheduler host test")

    # Scheduler is header-only; provide the tiny Arduino timing surface it needs
    # and exercise its behavior with a deterministic uint32_t clock.
    (tmp_path / "Arduino.h").write_text("#pragma once\n#include <cstdint>\nuint32_t micros();\n", encoding="utf-8")
    source = tmp_path / "scheduler_test.cpp"
    source.write_text(
        r"""
#include <cassert>
#include <cstdint>

#include "scheduler.hpp"

uint32_t g_now_us = 0;
uint32_t micros() { return g_now_us; }

namespace {
int g_runs = 0;

void run_task() { ++g_runs; }

void test_normal_schedule() {
  Scheduler<1> scheduler;
  g_now_us = 100;
  g_runs = 0;
  scheduler.add_task(100, run_task);

  g_now_us = 199;
  scheduler.tick();
  assert(g_runs == 0);
  g_now_us = 200;
  scheduler.tick();
  assert(g_runs == 1);
  g_now_us = 299;
  scheduler.tick();
  assert(g_runs == 1);
  g_now_us = 300;
  scheduler.tick();
  assert(g_runs == 2);
}

void test_blocked_scheduler_does_not_replay_debt() {
  Scheduler<1> scheduler;
  g_now_us = 0;
  g_runs = 0;
  scheduler.add_task(100, run_task);

  // Simulate a blocking reconnect attempt that lasts ten task periods.
  g_now_us = 1000;
  scheduler.tick();
  assert(g_runs == 1);
  for (g_now_us = 1001; g_now_us < 1100; ++g_now_us) {
    scheduler.tick();
  }
  assert(g_runs == 1);
  g_now_us = 1100;
  scheduler.tick();
  assert(g_runs == 2);
}

void test_rollover() {
  Scheduler<1> scheduler;
  g_now_us = 0xfffffff0u;
  g_runs = 0;
  scheduler.add_task(32, run_task);  // First deadline is 0x10 after rollover.

  g_now_us = 0x0fu;
  scheduler.tick();
  assert(g_runs == 0);
  g_now_us = 0x10u;
  scheduler.tick();
  assert(g_runs == 1);
  g_now_us = 0x2fu;
  scheduler.tick();
  assert(g_runs == 1);
  g_now_us = 0x30u;
  scheduler.tick();
  assert(g_runs == 2);
}

void test_delayed_resume_across_rollover() {
  Scheduler<1> scheduler;
  g_now_us = 0xffffffe0u;
  g_runs = 0;
  scheduler.add_task(64, run_task);  // First deadline is 0x20 after rollover.

  // Resume 256 us after that deadline: do not replay the four missed periods.
  g_now_us = 0x120u;
  scheduler.tick();
  assert(g_runs == 1);
  g_now_us = 0x121u;
  scheduler.tick();
  assert(g_runs == 1);
  g_now_us = 0x160u;
  scheduler.tick();
  assert(g_runs == 2);
}
}  // namespace

int main() {
  test_normal_schedule();
  test_blocked_scheduler_does_not_replay_debt();
  test_rollover();
  test_delayed_resume_across_rollover();
}
""",
        encoding="utf-8",
    )
    executable = tmp_path / "scheduler_test"
    compile_result = subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(tmp_path),
            "-I",
            str(REPO_ROOT / "firmware/include"),
            str(source),
            "-o",
            str(executable),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert compile_result.returncode == 0, compile_result.stderr

    run_result = subprocess.run([str(executable)], capture_output=True, text=True, check=False)
    assert run_result.returncode == 0, run_result.stderr
