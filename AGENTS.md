# Repository Guidelines
## Project Structure & Module Organization
- `Core/Src` holds application logic, peripheral init code, and FOC control routines; related headers live in `Core/Inc`.
- `Drivers/CMSIS` and `Drivers/STM32G0xx_HAL_Driver` mirror the STM32G0 HAL distribution; update them only through CubeMX exports.
- `MDK-ARM` contains the Keil uVision workspace (`FOC_TESTV1.0.uvprojx`), startup assembly, and generated HEX outputs.
- `FOC_TESTV1.0.ioc` plus `.mxproject` capture pin mapping and clock setup; regenerate code after hardware edits and review the diff in `Core` and `Drivers`.
- `.vscode` stores shared editor tasks; adjust absolute paths locally before committing.

## Build, Flash & Debug
- **Keil MDK**: open `MDK-ARM/FOC_TESTV1.0.uvprojx`, select the desired target, then `Project > Build Target`; flash via the ST-Link Download button.
- **STM32CubeIDE**: `File > Open Projects from File System`, point at `FOC_TESTV1.0.ioc`, then `Project > Build All` followed by `Run > Debug`.
- **CLI flashing**: export a HEX and run `STM32_Programmer_CLI -c port=SWD -d MDK-ARM/FOC_TESTV1.0/FOC_TESTV1.0.hex`.
- For VS Code, update `.vscode/launch.json` paths to match your build output before launching the C/C++ Runner config.

## Coding Style & Naming
- Use 4 space indentation, UTF-8 encoding, and keep CubeMX brace placement.
- Place custom logic inside `/* USER CODE BEGIN */` blocks so re-generation preserves changes.
- Follow HAL conventions: handles named `h<peripheral>` (for example `htim1`), public helpers in lower_snake_case, macros in UPPER_SNAKE_CASE.

## Testing & Validation
- No automated unit tests today; validate on bench hardware after each control or safety change.
- Capture USART telemetry for phase currents, DC bus voltage, and rotor speed; attach representative traces to review notes.
- When tuning control loops, log the procedure (input, load, expected response) in the merge request and keep parameter history in version control.

## Commit & Review Process
- Write imperative subject lines under 72 characters such as `Tune q axis current loop`; group unrelated changes into separate commits.
- Reference task IDs or issue numbers in the body (`Refs: FOC-123`) and enumerate any regenerated files.
- Pull requests require a short functional summary, affected modules, hardware prerequisites, and before/after measurements or screenshots.

## Configuration & Safety Notes
- Limit supply current with a clamp or programmable supply when bringing up new firmware.
- Keep release notes and the Keil target name aligned once validation completes.
