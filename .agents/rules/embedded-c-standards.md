# Embedded C Standards and Safety Rules

Source: [expert-embedded-c-engineer.agent.md](../../expert-embedded-c-engineer.agent.md)

## Standards and Context
- Target C99 as the baseline standard on Zephyr RTOS (ARM Cortex-M4F, nRF52840).
- Align with MISRA C:2012/2025 mandatory and required rules.
- Reference CERT C for security-sensitive paths.

## Code Design Rules
- Don't add abstractions unless they serve a clear purpose (testability, portability, or encapsulation).
- Don't default to global scope. Prefer file-scope (`static`) for internal functions and variables.
- Keep names consistent; follow project snake_case conventions.
- Comments explain **why**, not what.
- Don't add unused functions, parameters, variables, or includes.
- Use fixed-width integer types (`uint8_t`, `uint16_t`, `uint32_t`, `int8_t`, etc.) consistently.
- Wrap macro parameters in parentheses; wrap multi-statement macros in `do { ... } while(0)`.
- Use `const` qualification for pointers to read-only data, function parameters that should not be modified, and file-scope constants.
- Prefer `enum` over `#define` for related integer constants.

## Priorities
1. Correctness and standard compliance.
2. Safety (MISRA, CERT C, defensive checks).
3. Readability and maintainability.
4. Portability across compilers and targets.
5. Performance optimizations based on measured bottlenecks.
