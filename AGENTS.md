# AGENTS.md — BLE Smart Gate Controller

Shared high-level agent instructions, architecture guidelines, and workflows for Antigravity (`agy`) and Claude Code.

---

## 1. Project Overview & Controlling Document

* **Project:** BLE Smart Gate Controller (Sensor Fusion Redesign)
* **Target Hardware:** ProMicro nRF52840 (ARM Cortex-M4F)
* **Sensors:** HC-SR04 (bench prototype) / JSN-SR04T (recessed mount), NC Magnetic Reed Switch
* **Actuator:** 12 V Fail-Secure Electric Strike (auto-latch mechanics on close), 5V relay across intercom UNLOCK wires (connected NO)
* **Operating System:** Zephyr RTOS v4.4
* **System and Software Design Documentation:** [docs/BLE_Smart_Gate_System_and_Software_Design_Specification_L4B.pdf](docs/BLE_Smart_Gate_System_and_Software_Design_Specification_L4B.pdf) (Baseline L4-B, Revision 4)

---

## 2. Core Architecture & Fusion Invariants

1. **Refuse-by-Default Fusion:** Physical unlock commands fire **only** when all three conditions hold simultaneously:
   * **Identity:** Authenticated, bonded LE Secure Connection established.
   * **Presence:** Ultrasonic distance stream dwell $\ge$ `usUnlockDwellMs` (2.0 s).
   * **Position:** Reed switch is `CLOSED` (gate latched/locked).
2. **Unlock-Only Semantics:** The controller energizes the strike only to unlatch; upon `REED_OPEN`, relay IN drops `LOW` immediately to re-arm the strike. The mechanical auto-latch locks the gate upon closure.
3. **Fail-Secure Decoupling:** Any failure (power cut, WDT reset, sensor disconnection, software crash) leaves the strike de-energized (securely locked). The mechanical key always operates independently.
---

### 3. Safety & Concurrency
* **No Blocking on Workqueue:** Functions running on `sysworkq` must never call `k_sleep()` or block. Target per-item execution $< 5\text{ ms}$.
* **Hardware PPI/TIMER Capture:** Ultrasonic echo duration must be captured in hardware using `nrfx_timer` and `nrfx_gpiote` via PPI to prevent BLE radio interrupt jitter.
* **Float Rules:** FPU is single-precision only (`CONFIG_FPU=y`). Use `f` suffix on all float literals (e.g. `2.0f`). Floating-point math must run strictly on `sysworkq` (single thread, no `CONFIG_FP_SHARING`). Never use float in ISRs. Use float only if necessary
* **Flash (NVS) Safety:** `CONFIG_BT_CTLR_FLASH_GUARD=y` must be enabled. Do not call `nvs_write()` during active gate transit or on transient attempts; commit to flash only when entering permanent lockout (`BLOCKED`).

---

### 4. Code Organization & Style
* Target C99 standard. Follow MISRA C:2012/2025 and CERT C rules.
* **Scope Discipline:** Functions and variables internal to a `.c` file must be declared `static`.
* **Fixed-Width Types:** Use `uint8_t`, `uint16_t`, `uint32_t`, `int8_t`, `int32_t`, `bool` explicitly.
* **Error Handling:** Public module functions return explicit error codes (`int` or module enum: `0` / negative error code).
* **Logging:** Use Zephyr `LOG_INF`, `LOG_DBG`, `LOG_WRN`, `LOG_ERR` with deferred logging mode (`CONFIG_LOG_MODE_DEFERRED=y`). Never use `printk` in time-critical paths.

--- 
## 5. Installed Agents and Skills

* **Expert Embedded C Engineer:**
  * **Skill:** [`.agents/skills/expert-embedded-c-engineer/SKILL.md`](.agents/skills/expert-embedded-c-engineer/SKILL.md)
  * **Agent Definition:** [`.agents/agents/expert-embedded-c-engineer/agent.md`](.agents/agents/expert-embedded-c-engineer/agent.md)
  * **Coding Rules:** [`.agents/rules/embedded-c-standards.md`](.agents/rules/embedded-c-standards.md)
  * **Standard:** C99, MISRA C:2012/2025, CERT C, pointer discipline, volatile correctness, defensive programming.

--- 

## 6. Build & Flash Commands
```bash
# Build firmware using the project script:
./scripts/build_and_flash.sh

# Or standard Zephyr west build:
west build -b adafruit_feather_nrf52840 -s . -d build -- -DEXTRA_DTC_OVERLAY_FILE=app.overlay

# Clean build:
west build -t clean
```

---
