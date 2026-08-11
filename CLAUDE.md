# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Raspberry Pi robot controller (RoboCup ring match): differential chassis (2x CDS servos) + sensors (4 analog ADC grayscale, analog/digital IR, digital IO input) + behavior strategy. Code comments are in Chinese; match that convention.

## Development rules (user-mandated — do not skip)

1. **Never modify the driver layer**: `uptech.py` and `up_controller.py` are the hardware driver wrapper and must not be edited (wiring/direction fixes go in new modules or config).
2. **Grill before coding**: run `Skill(mattpocock-skills:grilling)` first to settle design details layer by layer (data collection → algorithm choice → implementation) before writing code.
3. **Follow karpathy-guidelines while coding**: load `Skill(karpathy-guidelines)` before each development session; simplicity first, no speculative abstraction, surgical changes.
4. **Production/dev separation**: `main.py` is the only production hardware entry. Root strategy and sensor modules contain only injected-data logic and must not import `uptech`/`up_controller`, expose hardware or calibration CLIs, or use `DEV_MODE`. Real-hardware strategy runners and sensor collection/calibration tools live under `dev/`.
5. **Collected data must be saved to a file** (CSV) for calibration/analysis — never hardcode values from memory.
6. **config.py aggregates module parameters**: it was deleted; rebuild it from module parameters as modules are written (channel mapping, thresholds, sample rate, file paths).
7. **Never end on "tests pass"**: after implementing, explain the verification plan (how to validate on real hardware / on the bench).
8. **Do not trust deleted config remnants**: old thresholds/channel mappings/normalization are void — re-derive everything from fresh collected data.

## Architecture

- `uptech.py` — vendor Raspberry Pi hardware library (`UpTech`; ctypes to `libuptech.so` + pigpio): CDS servo speed/mode, 10-ch ADC read, IO input mask, LCD. **Do not edit.**
- `up_controller.py` — driver wrapper: `move_cmd(left, right)` (CDS id 7 = left wheel, id 8 = right wheel negated; software `motor_invert`/`motor_swap` fixes), background poll thread filling `adc_data` (10 ch) / `io_data` (8-bit mask), `stale()`/`healthy` health flags, `close()` safe shutdown (stop motors → stop thread → close hardware). Raises `RuntimeError` without `uptech` (PC dev uses stubs). **Do not edit.**
- `main.py` — production coordinator and the only match entry: updates reentry first; any reentry state other than `WAIT` preempts patrol and owns the motor command. Patrol is recreated after reentry releases control so stale timers cannot resume.
- `ring_patrol.py` / `reentry.py` — pure strategy state machines; receive externally injected sensor data and return motor commands; no hardware ownership or CLI.
- `dev/ring_patrol.py` / `dev/reentry.py` — independent real-hardware strategy tests and CSV logging; reentry runner also supports PC CSV replay.
- `dev/gray_tool.py` / `dev/ir_tool.py` / `dev/digi_ir_tool.py` — sensor scan and CSV collection tools; digital IR collection saves both raw IO bits and mapped states.
- `dev/calibrate_gray.py` — offline gray-model calibration from files already stored under `data/`.
- `tests/test_<module>.py` — PC automated unit and replay tests, run through `unittest discover`.
- `tests/motor_test.py` — interactive real-hardware motor test; requires explicit safety confirmation and is not discovered as a unit test.
- Peripheral modules (`gray.py`, `digi_ir.py`, `ir.py`) — pure sensor conversion and injected-data interfaces; no hardware ownership or CLI.
- `config.py` — parameter aggregation for the modules.

## Environment

- Hardware runs only on the Raspberry Pi (needs `libuptech.so` + pigpio); on PC only stub/injected-data logic runs.
- `python3 main.py` — production match program (patrol + reentry, real hardware).
- `python3 dev/ring_patrol.py` — patrol-only real-hardware test and CSV log.
- `python3 dev/reentry.py [--force-trigger]` — reentry-only real-hardware test and CSV log.
- `python3 dev/reentry.py --replay data/<file>.csv` — PC gray CSV replay.
- `python3 dev/gray_tool.py scan|collect ...` — gray scan or CSV collection.
- `python3 dev/ir_tool.py scan|collect ...` — analog IR scan or CSV collection.
- `python3 dev/digi_ir_tool.py scan|collect ...` — digital IR scan or CSV collection.
- `python3 dev/calibrate_gray.py` — rebuild gray calibration outputs from `data/`.
- `python3 tests/motor_test.py <action> [--ground]` — interactive motor direction/speed test.
- `python3 up_controller.py` — driver-level sensor direct-read self-test (real hardware).
- PC verification uses `python3 -m unittest discover -s tests -t . -v`; real-hardware verification remains mandatory for motor direction, state transitions, and recovery distance.
