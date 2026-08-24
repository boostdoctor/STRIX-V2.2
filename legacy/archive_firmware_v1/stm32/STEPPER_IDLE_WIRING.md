# Stepper Idle Valve — Wiring Diagram (TorquEFI / STRIX)

When **Drive-by-wire is disabled** and **Idle output = Stepper (4-wire)**  
(`SET:DBW,0` + `SET:IDLEOUT,2`).

Typical part: **4-wire bipolar IAC / idle air control stepper** (e.g. GM-style or generic NEMA-like IAC).

---

## 1. Overview

```
                    +12V (ignition-switched, fused 3–5 A)
                         |
                    [Flyback / TVS on supply]
                         |
              ┌──────────┴──────────┐
              │  Stepper driver     │
              │  (e.g. A4988,       │
              │   DRV8825, or       │
              │   dual H-bridge)    │
              │                     │
   STM32      │   OUT1A ── coil A+  │
   3.3 V ─────┤   OUT1B ── coil A−  │──── 4-wire
   logic      │   OUT2A ── coil B+  │      stepper
              │   OUT2B ── coil B−  │      IAC valve
              │                     │
              │   GND ──────────────┼──── ECU / chassis GND
              └─────────────────────┘
```

**Never drive the motor coils directly from the STM32 pins.**  
Use a stepper driver or dual H-bridge rated for **12 V** and coil current.

---

## 2. Recommended MCU → driver interface

| Driver input | Black Pill pin | Notes |
|--------------|----------------|--------|
| **STEP** | **PA8** | Was ETB PWM — step pulse |
| **DIR** | **PA9** | Was ETB DIR — direction |
| **ENABLE** (optional) | **PB8** | Shared with boost if unused; or hard-enable |
| **GND** | GND | Common with driver |

```
    STM32F411                    Stepper driver              IAC stepper
   ┌─────────┐                  ┌──────────────┐           ┌──────────┐
   │         │   STEP           │              │  A+       │          │
   │  PA8 ───┼─────────────────►│ STEP         ├──────────►│ Coil A   │
   │         │   DIR            │              │  A−       │          │
   │  PA9 ───┼─────────────────►│ DIR          ├──────────►│          │
   │         │   ENABLE (opt)   │              │  B+       │ Coil B   │
   │  PB8 ───┼─────────────────►│ ENABLE       ├──────────►│          │
   │         │                  │              │  B−       │          │
   │  GND ───┼─────────────────►│ GND          ├──────────►│          │
   │         │                  │ VMOT ◄── +12V│           └──────────┘
   └─────────┘                  │ GND  ◄── GND │
                                └──────────────┘
```

---

## 3. 4-wire bipolar coil identification

```
        Motor connector (typical)
        ─────────────────────────
              ┌─────┐
         A+ ──┤1   4├── B+
         A− ──┤2   3├── B−
              └─────┘

   Pair coils with a multimeter (low ohms = one winding):

        A+ ●─────────● A−     winding A
        B+ ●─────────● B−     winding B
```

If the valve moves the wrong way, swap **DIR** logic in software or swap one coil pair.

---

## 4. Power & protection

| Item | Recommendation |
|------|----------------|
| Supply | **+12 V** switched (IGN), **3–5 A fuse** |
| Driver VMOT | 12 V (or per motor rating) |
| Logic | 3.3 V or 5 V per driver (A4988 often 5 V logic — level shift if needed) |
| Flyback | Driver usually includes it; still use supply TVS |
| Ground | MCU GND = driver GND = battery negative |

```
   Battery + ──[FUSE]──[IGN switch]── +12V rail ── driver VMOT
   Battery − ─────────────────────── GND ──────── driver GND / MCU GND
```

---

## 5. Software mode

```text
SET:DBW,0
SET:IDLEOUT,2
SET:IDLE,850
SET:IDLEEN,1
```

- Idle engages only when **TPS &lt; 5%**
- STEP/DIR on **PA8/PA9** (same pins as 2-wire DBW/H-bridge path)
- Full step sequencer in firmware is a future refinement; hardware wiring above is the target

---

## 6. vs other idle modes

| Mode | `SET:IDLEOUT` | Wiring |
|------|---------------|--------|
| **2-wire PWM** | 0 | H-bridge: PA8 PWM, PA9 DIR → DC motor IAC |
| **1-wire PWM** | 1 | Low-side MOSFET: PA8 PWM → solenoid, +12 V other side |
| **Stepper** | 2 | Driver STEP/DIR as above → 4-wire bipolar |

---

## 7. Safety

- Do not power the stepper from USB / 3.3 V  
- Confirm coil resistance before connecting (typically a few Ω to tens of Ω)  
- Set driver current limit to motor rating  
- If the throttle is still a cable throttle, the stepper only bypasses air at closed throttle  

