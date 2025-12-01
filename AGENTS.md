## Language Preference
- 所有解释、注释、推理和输出一律使用简体中文回复。
- 遇到技术术语可在中文后括号保留英文（如需）。

# Repository Guidelines

## Project Structure & Module Organization
- Firmware sources live in `Core/Src`; headers in `Core/Inc` keep peripheral APIs close to implementation.
- Vendor libraries and HAL drivers sit under `Drivers/CMSIS` and `Drivers/STM32F4xx_HAL_Driver`; avoid editing generated vendor code.
- Keil uVision project files are in `MDK-ARM` (`shoutao.uvprojx`, `.uvoptx`, startup). Build artifacts and map/hex files land in `MDK-ARM/shoutao`.
- Board-level configuration and middleware are organized inside `MDK-ARM/HARDWARE`; keep board-specific changes there instead of Core.

## Build, Test, and Development Commands
- GUI build/flash: open `MDK-ARM/shoutao.uvprojx` in Keil uVision, choose the desired target, then Build and Download to flash.
- CLI build (when `UV4.exe` is on PATH): `UV4.exe -b MDK-ARM\shoutao.uvprojx -j0 -o MDK-ARM\build.log`; artifacts remain in `MDK-ARM\shoutao`.
- Regenerate peripheral init code from `shoutao.ioc` using STM32CubeMX, exporting to existing `Core` and `Drivers` paths to keep references intact.

## Coding Style & Naming Conventions
- Language: C with STM32 HAL. Use 4-space indentation and keep lines under ~120 chars.
- Naming: functions/variables `snake_case`, macros/constants `UPPER_SNAKE_CASE`, files lower_snake_case to match module names.
- Prefer `static` helpers inside their module; include order: module header, standard headers, HAL headers.
- Avoid manual edits inside CubeMX-managed blocks; add custom code only in user sections.

## Testing Guidelines
- No automated test harness; rely on bench verification. Document inputs and observed UART output for each run.
- Before committing, run a clean build in Keil and confirm the generated hex/elf matches the intended target.
- For peripheral changes, probe signals (scope/logic analyzer) and log register-level observations in the PR description.

## Commit & Pull Request Guidelines
- Git history shows short, imperative commit titles (often in Chinese). Follow that style, e.g., "增加 USART 超时保护" or "Fix encoder read timing".
- Keep commits small and focused; avoid committing large binaries unless required for release.
- PRs should link related issues (if any), summarize the change, list manual test steps/results, and attach relevant screenshots or serial logs.

## Security & Configuration Tips
- Do not commit secrets, license keys, or board-specific credentials. Keep personal `.uvguix` or IDE metadata out of commits unless intentionally shared.
- Verify `.gitignore` covers local build caches; clean `MDK-ARM/shoutao` outputs before publishing release branches.
