# CLAUDE.md — Claude Code Project Guidelines

## Project Context
* **Project:** BLE Smart Gate Controller
* **SoC / Architecture:** Nordic Semiconductor nRF52840 (ARM Cortex-M4F with FPU)
* **RTOS:** Zephyr RTOS v4.4
* **Controlling Spec:** `docs/BLE_Smart_Gate_System_and_Software_Design_Specification_L4B.pdf` (L4-B Rev 4)

---

## Build & Flash Commands
```bash
# Build firmware using the project script:
./scripts/build_and_flash.sh

# Or standard Zephyr west build:
west build -b adafruit_feather_nrf52840 -s . -d build -- -DEXTRA_DTC_OVERLAY_FILE=app.overlay

# Clean build:
west build -t clean
```

---

## Coding Standards & Critical Constraints

### 1. Safety & Concurrency
* **No Blocking on Workqueue:** Functions running on `sysworkq` must never call `k_sleep()` or block. Target per-item execution $< 5\text{ ms}$.
* **Hardware PPI/TIMER Capture:** Ultrasonic echo duration must be captured in hardware using `nrfx_timer` and `nrfx_gpiote` via PPI to prevent BLE radio interrupt jitter.
* **Asymmetric Reed Debouncing:** 30 ms on contact break (OPEN), 150 ms on contact make (CLOSE).
* **Float Rules:** FPU is single-precision only (`CONFIG_FPU=y`). Use `f` suffix on all float literals (e.g. `2.0f`). Floating-point math must run strictly on `sysworkq` (single thread, no `CONFIG_FP_SHARING`). Never use float in ISRs.
* **Flash (NVS) Safety:** `CONFIG_BT_CTLR_FLASH_GUARD=y` must be enabled. Do not call `nvs_write()` during active gate transit or on transient attempts; commit to flash only when entering permanent lockout (`BLOCKED`).

### 2. Code Organization & Style
* Target C99 standard. Follow MISRA C:2012/2025 and CERT C rules.
* **Scope Discipline:** Functions and variables internal to a `.c` file must be declared `static`.
* **Fixed-Width Types:** Use `uint8_t`, `uint16_t`, `uint32_t`, `int8_t`, `int32_t`, `bool` explicitly.
* **Error Handling:** Public module functions return explicit error codes (`int` or module enum: `0` / negative error code).
* **Logging:** Use Zephyr `LOG_INF`, `LOG_DBG`, `LOG_WRN`, `LOG_ERR` with deferred logging mode (`CONFIG_LOG_MODE_DEFERRED=y`). Never use `printk` in time-critical paths.

### 3. Agent Skills Reference
Detailed embedded C guidelines are available in:
* `.agents/skills/expert-embedded-c-engineer/SKILL.md`
* `.agents/agents/expert-embedded-c-engineer.agent.md`
* `.agents/rules/embedded-c-standards.md`
