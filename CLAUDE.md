# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Raspberry Pi robot controller (RoboCup ring match): differential chassis (2x CDS servos) + sensors (4 analog ADC grayscale, analog/digital IR, digital IO input) + behavior strategy. Code comments are in Chinese; match that convention.

## Development rules (user-mandated — do not skip)

1. **Never modify the driver layer**: `uptech.py` and `up_controller.py` are the hardware driver wrapper and must not be edited (wiring/direction fixes go in new modules or config).
2. **Grill before coding**: run `Skill(mattpocock-skills:grilling)` first to settle design details layer by layer (data collection → algorithm choice → implementation) before writing code.
3. **Follow karpathy-guidelines while coding**: load `Skill(karpathy-guidelines)` before each development session; simplicity first, no speculative abstraction, surgical changes.
4. **Module separation + dev mode**: each peripheral module is standalone — runs independently, does not import other project modules; a same-file define constant (`DEV_MODE`) switches dev/production: dev = reads hardware directly for self-test, production = external data injection (unit-testable on PC).
5. **Collected data must be saved to a file** (CSV) for calibration/analysis — never hardcode values from memory.
6. **config.py aggregates module parameters**: it was deleted; rebuild it from module parameters as modules are written (channel mapping, thresholds, sample rate, file paths).
7. **Never end on "tests pass"**: after implementing, explain the verification plan (how to validate on real hardware / on the bench).
8. **Do not trust deleted config remnants**: old thresholds/channel mappings/normalization are void — re-derive everything from fresh collected data.

## Architecture

- `uptech.py` — vendor Raspberry Pi hardware library (`UpTech`; ctypes to `libuptech.so` + pigpio): CDS servo speed/mode, 10-ch ADC read, IO input mask, LCD. **Do not edit.**
- `up_controller.py` — driver wrapper: `move_cmd(left, right)` (CDS id 7 = left wheel, id 8 = right wheel negated; software `motor_invert`/`motor_swap` fixes), background poll thread filling `adc_data` (10 ch) / `io_data` (8-bit mask), `stale()`/`healthy` health flags, `close()` safe shutdown (stop motors → stop thread → close hardware). Raises `RuntimeError` without `uptech` (PC dev uses stubs). **Do not edit.**
- Peripheral modules (`gray.py`, …) — added per rule 4: independent dev self-test entry + production data-injection interface.
- `config.py` — parameter aggregation for the modules.

## Environment

- Hardware runs only on the Raspberry Pi (needs `libuptech.so` + pigpio); on PC only stub/injected-data logic runs.
- `python3 up_controller.py` — sensor direct-read self-test (dev mode, real hardware).
- No test/lint infra yet; new modules verify on two tracks: PC (injected data) + real-hardware dev self-test.
