# CARL battery monitor — concept handoff for a future ChatGPT chat

> Purpose: Upload this file into a new CARL project chat when ready to begin the KiCad project. Treat it as a record of the design discussion, not a finished schematic, verified battery datasheet, or authorization to fabricate hardware. The user is learning KiCad and PCB design; explain design decisions and the underlying electrical theory as the project progresses.
>
> Status as of 2026-11-18: Functional concept agreed. The user is preparing a separate electrical interface document and **will not start the PCB project until that document is complete**. Incorporate that document when supplied; it should resolve physical connections and may supersede tentative details here.

## Objective and boundaries

Build an original, battery-mounted monitor for CARL, using Victron's SmartShunt as a **functional and performance reference**, with native wired CAN telemetry instead of wireless communication. Measure whole-pack terminal/output voltage and total signed current; estimate state of charge (SOC) from measured charge flow; report measurements, validity, and faults. The battery's internal BMS remains responsible for cell-level protection. No cell taps or external temperature sensors are in scope. The monitor must not directly command vehicle behavior or act as a second electronic high-current cutoff; CARL's STM32 NUCLEO-F767ZI master owns responses to telemetry and faults.

This is a custom PCB and firmware learning project, not a plan to install a Victron SmartShunt or a VE.Direct-to-CAN bridge. Victron's published behavior/specifications are benchmarks, not evidence of its undisclosed internal schematic or code.

## Known system facts and agreed decisions

| Topic | Current understanding |
| --- | --- |
| Battery | Ingeosolly `INGO-24V50` as identified by the user; LiFePO4, 25.6 V nominal, 50 Ah nominal, 1280 Wh nominal, rated 50 A output. Exact manufacturer charge-current limit, BMS cutoff, and peak-current duration have not been established from an authoritative pack datasheet. |
| Motor budget | Four motors. User-selected concept assumption: **40 A combined** motor draw in a demanding case. Other vehicle loads also draw from the pack. Design target is at least 50 A total **continuous** current path. Actual startup/stall peaks remain unknown. Blown 7.5 A individual motor-branch fuses do not set a maximum peak. |
| Placement | Monitor assembly remains attached to the battery, envisioned on top of it, and travels with it for off-vehicle charging. Short battery leads. Final enclosure, mounting, clearance, and cable strain relief need the user's interface document. |
| Main connection | One shared high-current `OUT+`/`OUT−` connector is used alternately for CARL and the charger. Charger and vehicle must both connect on the system side of the shunt; charging at the bare battery terminals bypasses current measurement. |
| Vehicle power | CARL has +24 V and GND bus bars. An existing inline 50 A fuse with manual switch lies between battery positive and +24 V bus. Motor loads and converters are downstream. A future power distribution board is expected to provide 12 V, 5 V, 3.3 V, but these switched rails are off with CARL. |
| Monitor power | The module must remain powered from the battery while CARL is off and while charging off-vehicle. Thus it needs a protected, low-standby battery-to-logic supply; relying solely on CARL's 3.3 V rail is no longer the plan. Vehicle CAN electronics can sleep or be gated when disconnected. |
| Programming | MCU embedded directly in PCB; permanent SWD header and UART factory-bootloader service access (including reset and appropriate boot selection). Firmware updates only through service connections, not CAN. Exact STM32 boot pins/option bytes depend on the selected part. |
| CAN | Wired telemetry over CARL's classic CAN 2.0, proposed 500 kbit/s, standard 11-bit IDs. External transceiver, detachable CAN connector/screw terminals, switchable 120 Ω termination enabled only at a physical end of the cable. No wireless requirement. |
| Operator control | Physical, labeled **SET FULL** button, preferably guarded by a deliberate long press and visible confirmation, for manual synchronization after a confirmed full charge. |
| Accuracy | SOC displayed/reported in 1% steps; aspiration is within a few percentage points over repeated trips/cycles after calibration and periodic synchronization. Resolution is not accuracy. Validate empirically. |

## Working electrical topology (not a schematic)

```text
Battery + ── short protected lead ── module fuse ── four-terminal shunt ── OUT+
Battery − ──────────────────────────────────────────────────────────── OUT−
                                         │
          battery-side protected low-power supply → MCU and measurement IC
          Kelvin shunt sense + battery voltage sense → measurement IC → MCU
          MCU → CAN transceiver → detachable CAN_H/CAN_L/reference connection
          MCU → SWD, bootloader UART, SET FULL button, status indication

Use OUT+/OUT− for either:
  (a) vehicle harness → CARL's existing fuse/manual switch and bus bars, or
  (b) charger harness → off-vehicle charger.
```

The working shunt location is on the **positive/high side**, permitting the battery and vehicle grounds to share a direct reference. An external/bolted, four-terminal shunt with a physically associated sensing PCB is the preferred starting construction; whether a rated high-current PCB/busbar assembly can satisfy size and thermal constraints is unresolved. Both polarities of the output connector and any passthrough conductor must support the continuous and transient current. Ordinary signal terminals/traces are not automatically suitable. Kelvin sensing avoids measuring voltage drops in the high-current terminals and conductors. A tentative 0.5 mΩ shunt gives 20 mV/0.8 W at 40 A and 25 mV/1.25 W at 50 A; this is an **example, not an approved part or thermal design**.

The monitor's own supply drawn directly from battery-side positive may bypass the shunt. Its small quiescent current should be measured and accounted for in the SOC error budget. Define whether the reported voltage is battery-side or output-side; useful options are battery-side pack voltage plus an output/fuse-status sense. Avoid assuming that a single voltage reading proves the fuse or switch state.

### High-current connector and fuse caveats

- User suggested XT90S for anti-spark behavior, believing it is 90 A rated. A published [Amass XT90S-F specification](https://datasheet.lcsc.com/datasheet/pdf/486179007ba2562b1fdba2d94c5d08.pdf?productCode=C3040691) lists **40 A rated/90 A momentary** for that variant. It is **not selected** for this 50 A continuous path. Select an exact keyed, touch-safe-as-practical connector assembly using its continuous current, cable/termination, contact heating, mating-cycle, and transient ratings. Anti-spark precharge is not overcurrent protection.
- User wants a replaceable, lab-available automotive blade-type fuse in a PCB-mounted slot. The exact fuse family, holder, DC voltage rating, interrupting capability against battery short current, heat, accessibility, and mechanical mounting remain open. A 50 A [Littelfuse MAXI](https://www.littelfuse.com/assetdocs/littelfuse_maxi32v_datasheet.pdf?assetguid=68b8bbe8-3e51-4c7b-bbda-3c71667f75f3) is an *example family* rated 32 V DC, not an approved selection. Verify margin against pack and transients. The battery-to-module segment upstream of the module fuse is not protected by that fuse; keep it short/protected and resolve whether a battery-terminal fuse is required. Coordinate the module fuse with CARL's existing switched 50 A fuse.
- The module is a monitor with a fused output, not an active bidirectional output disconnect. Protect its sensing and logic inputs against reverse connection, overvoltage, motor transients, and wiring faults; detect/report out-of-range conditions. A small TVS or logic-side polarity protector cannot clamp or disconnect an arbitrary high-energy battery fault from all vehicle loads. The pack BMS, correctly rated fuses, wiring, and master response retain their respective roles.

## Charger and SOC behavior

Preferred charger candidate: **Victron Blue Smart IP65 24/8**, used in its physical-button-selectable Li-ion/LiFePO4 mode. Victron documents 8 A maximum (3 A low-current mode), 28.4 V Li-ion absorption, and 27.0 V storage. The user believes the pack permits the 28.4 V profile. **An authoritative confirmation of this exact pack's allowed charge current, voltage, and temperature range is still outstanding.** The 50 A discharge rating does not prove charging limits. The 24/8 is a candidate, not a completed compatibility certification. See [Victron charger specifications](https://www.victronenergy.com/media/pg/Blue_Smart_IP65_Charger_120V_manual/en/technical-specifications---bsc---psc.html) and [charge modes](https://www.victronenergy.com/media/pg/Blue_Smart_IP65_Charger_120V_manual/en/operation---bsc---psc.html).

With charger connected through `OUT`, measure signed current and integrate charge/discharge in amp-hours against *calibrated usable capacity*, starting from a trustworthy SOC reference. Victron-like automatic full synchronization requires: (1) a charger-appropriate voltage threshold near the end-of-charge stage, (2) observed **charging** current tapered below a configured threshold, and (3) both conditions sustained for a configured duration. Do not hard-code an unverified threshold or treat zero current/high voltage as full; characterize the actual pack/charger cycle and guard against false synchronization as the charger transitions to storage. [Victron SmartShunt method](https://www.victronenergy.com/media/pg/SmartShunt/en/operation.html).

If someone charges directly at the battery's original terminals, current bypasses the shunt. The powered monitor may observe voltage change and flag **possible unmeasured charge**, but it cannot infer an accurate final SOC from pack voltage alone (the LiFePO4 mid-range voltage curve is flat). Mark SOC **uncertain** until a confirmed full charge is synchronized with the SET FULL button or another defensible reference. Normal power cycles should restore saved SOC. A partly charged battery with unknown off-module history should show SOC unknown or explicitly operator-estimated, never silently 100%. Battery voltage/current can still be reported when SOC is invalid.

Persist SOC and validity with bounded-write storage (implementation choice TBD). Checkpoint periodically and on significant changes; a GUI `STOP CARL` action may request an extra save/acknowledgment, but manual switch-off, unplugging, or BMS cutout must not make that request the sole persistence mechanism. Define behavior after storage corruption, brownout, a fuse opening, and a boot with no valid SOC reference. An explicit battery identifier may be useful if packs become interchangeable.

## Electronics and communications concept

- **MCU candidate:** STM32G0B1 family for I2C, USART, SWD, watchdog, nonvolatile storage, and FDCAN configured in classic CAN mode. STM32G4 is capable but presently lacks a demonstrated benefit with a dedicated digital measurement IC. Select exact package/pins, flash endurance strategy, and factory UART bootloader routing against ST's datasheet and AN2606 before schematic capture.
- **Measurement candidate:** TI INA228 or comparable high-side, bidirectional shunt and bus-voltage monitor, I2C to MCU. Check input common-mode and differential limits, transients, power draw, accuracy across the desired range, and off-state behavior. [INA228 datasheet](https://www.ti.com/lit/ds/symlink/ina228.pdf). No part is finalized.
- **Power:** protected battery-side low-power conversion with attention to standby draw and CAN backfeeding when the module is removed from CARL. Victron publishes <1 mA total draw as a *comparison benchmark*, which may be difficult for an STM32 + measurement IC + regulator without sleep/duty-cycle design. Test actual draw and translate it into lost Ah over storage time. [SmartShunt specifications](https://www.victronenergy.com/media/pg/SmartShunt/en/technical-data.html).
- **CAN:** telemetry-only native classic CAN. Define message ID(s), scaling, signed current convention, period, SOC-valid/uncertain indication, fault bits, and timeout behavior with CARL's existing [CAN specification](can-bus.md). The current master forwards only selected received IDs to the companion, so update that path and later GUI integration when firmware work begins. Physically CAN_H/CAN_L are a shared twisted pair with short node stubs and exactly two endpoint terminators, regardless of which node is the master.
- **Service:** permanent SWD and UART bootloader access, including reset, ground, voltage reference, and boot selection as required by the exact STM32. CAN does not carry firmware updates.
- **Fault role:** report electrical/measurement faults to master; no local commands to throttle/steering, no electronic high-current cutoff. Define which observed conditions are warnings versus measurement-invalid, using verified battery limits where necessary.

## Benchmark and validation intent

Use Victron SmartShunt's published specifications as comparison points: current measurement accuracy ±0.4%, voltage measurement accuracy ±0.3%, SOC display resolution 0.1%, standby draw <1 mA. These figures are **not promises for CARL**, and display resolution is not SOC accuracy. Our target is 1% SOC reporting steps and an empirically measured error of a few percentage points over repeated trips/cycles. Plan calibration and tests at zero current, representative motor loads, 50 A continuous-equivalent conditions where safe/testable, charging near 8 A, shutdown/restart, charge completion, bypass charge, fuse opening, CAN loss, and brownout. Verify voltage against a calibrated meter and current/Ah against a calibrated instrument or reference monitor.

## Next work, in order

1. Wait for and incorporate the user's **electrical interface document**. Resolve battery top geometry, mounting, upstream lead length/protection, power connector and charger mating cable, fuse/fuse-holder construction, shunt installation, CAN harness, and service access. Do not create a KiCad project before the user completes that document.
2. Write a reviewed requirements/connector and power-path specification with explicit known, assumed, and unresolved values. Resolve any conflict between the interface document and this handoff with the user.
3. Select and verify exact parts from manufacturer documentation, with a power/current/thermal budget and fault-path analysis. Preserve the user's preference for a replaceable automotive fuse if a suitable rated fuse/holder assembly can be found.
4. In KiCad, teach while implementing: block diagram → schematic sheets → symbols and footprints → ERC → placement/current path/thermal and Kelvin routing → DRC → fabrication outputs → bring-up and calibration plan. Explain why each block and check is needed. Do not jump directly to board layout.

Do not infer that all unresolved electrical ratings or physical constraints are approved merely because the functional concept is agreed.
