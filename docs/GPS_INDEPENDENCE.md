# PUSHPAK · GPS independence

What "GPS-denied" means in PUSHPAK, how much of it is real today, and what has to be built before the claim holds on hardware.

## Why it matters

Satellites keep working after an earthquake. What fails is reception: under a slab, inside a void or deep between damaged buildings, a receiver loses the signal or gets reflections. People are trapped exactly where GNSS cannot reach. The scout that goes in after them (Drone B) must therefore hold position and search with no satellite fix at all. The survey drone above the site (Drone A) has open sky most of the time and should use GNSS when it is healthy, while still surviving without it.

## Drone B: the claim and how far it goes

**Claim:** Drone B navigates with no GPS, no compass and no absolute yaw source.

| Level | Requirement | Status today |
|---|---|---|
| 1 | No GNSS anywhere in the navigation loop | **Done in software.** `GPS_TYPE 0`, `COMPASS_USE 0`; EKF3 runs on external navigation from our estimator. EKF3 needs an arbitrary origin to run a local frame; it is set once and no satellite data is used (scaffold S-4). |
| 2 | Horizontal motion measured on board | **Emulated.** Velocity comes from a radar emulator: simulator body velocity plus noise. No real radar has been integrated. |
| 3 | Height measured on board | **Simulated sensor.** A 1D ToF rangefinder model (≤ 4 m). |
| 4 | Drift reported, never hidden | **Done.** Drift = bias × time, measured to within 0.1 % and 0.9 % (hover gate, noise removed); the dashboard shows the estimate, not truth. |
| 5 | Velocity bias detected in flight | **Not built.** A constant bias is invisible to the filter (`docs/ODOMETRY_ANALYSIS.md` §2). |
| 6 | Heading held without a compass | **Not built.** Heading is integrated from the gyro; the simulated gyro has no bias, so the 0.02° per minute drift is optimistic. |
| 7 | Absolute position resets | **Not built.** UWB anchors at the entrance are designed. |
| 8 | All of the above on flight hardware | **Not built.** |

So: GPS-free in the software loop and measured in simulation (levels 1–4); not yet GPS-independent in the field (levels 5–8). The 0.58 m mean error over a 62.3 s search is a result about the pipeline under an emulated sensor.

## How the estimate is made today

```
IMU 200 Hz ─────────┐
radar velocity 20 Hz ├─► robot_localization EKF (15 states, 50 Hz) ─► /odometry/filtered ─┬─► pushpak_brain (guidance)
ToF height 30 Hz ───┘                                                                  └─► vio_bridge_node ─► ArduPilot EKF3 (external nav)
```

ArduPilot is configured with external navigation as its only horizontal and yaw source (`EK3_SRC1_POSXY 6`, `EK3_SRC1_VELXY 6`, `EK3_SRC1_YAW 6`) and the rangefinder for height (`EK3_SRC1_POSZ 2`; README §11).

### Four known weaknesses

1. **Two filters in series, IMU used twice.** robot_localization fuses the IMU without bias states and hands its output to EKF3 as a "vision" measurement; EKF3 fuses the same IMU again. The two errors are correlated, so EKF3 is over-confident.
2. **The radar is emulated.** Truth plus white noise produces a clean, unbiased velocity. A real radar needs an ego-velocity estimator and brings bias, scale-factor and mounting errors.
3. **No bias state for external velocity.** EKF3 has no state for a bias in external-navigation velocity, and the velocity covariance does not grow under a constant bias. Tr(Σv) catches a dead sensor, not a biased one.
4. **Heading and height.** Heading drifts with gyro bias, which near rebar and cables cannot be corrected by a magnetometer. The ToF measures the nearest slab, tops out at 4 m and weakens in dust.

## The committed design for Drone B

None of this is built; each item answers one weakness above.

| Item | Design | Answers |
|---|---|---|
| One filter | Radar ego-velocity and the rangefinder fused directly in ArduPilot EKF3; retire `ekf_node` and `vio_bridge_node`; optical-flow velocity as the fallback source | 1 |
| Real radar velocity | TI IWR6843: estimate ego-velocity from the Doppler of static returns with a robust least-squares fit (RANSAC), following Kramer et al. (ICRA 2020) | 2 |
| Bias detection | Compare radar velocity with optical-flow velocity: they fail for different physical reasons (radar: multipath, mounting; flow: dust, darkness, texture). Disagreement says something is wrong, not which sensor, so the drone holds or lands | 3 |
| Absolute resets | UWB anchors at the entrance reset position whenever the drone is in range; two anchors are GPS-surveyed so pins come out in latitude and longitude | 3 |
| Heading | Two UWB tags on the airframe, or a heading fit from a straight leg between two UWB fixes; gyro integration in between | 4 |
| Height | Smoothed height hold over rubble; radar range as a backup to the ToF | 4 |
| Bias-aware filter (research) | Radar-inertial odometry with bias states, the approach of Doer & Trommer (MFI 2020), as the longer-term estimator | 3 |

### The drift budget that sets the design

With a velocity bias b the error grows as b × t. Holding 1 m therefore lasts 20 s at 0.05 m/s and 10 s at 0.10 m/s. Until the real radar's bias is measured on the bench, the design assumes:

- short sorties, with UWB resets each time the drone passes within range of the entrance anchors;
- the radar vs optical-flow check running continuously, so a growing bias is caught while flow is valid;
- relay nodes dropped at void entrances, so the drone can retrace to a known point when the link or the estimate degrades.

## Drone A: GNSS when healthy, flying without it when not

Drone A surveys from 30–50 m. Up there it usually has open sky, so GNSS is the best position source and the design uses it. It must still keep flying when GNSS is degraded, for example by reflections between tall damaged buildings or by interference.

**Mechanism.** ArduPilot's EKF3 supports up to three sets of navigation sources (position, velocity, height and yaw each chosen per set) and can switch between them in flight, either from an RC switch or from a Lua script that watches sensor quality. ArduPilot ships example scripts that switch between GPS and optical flow using measures such as GPS speed accuracy and EKF innovations.

| | Source set 1 (primary) | Source set 2 (fallback) |
|---|---|---|
| Horizontal position and velocity | GNSS | downward camera: optical flow or visual odometry on the Raspberry Pi 5 (to be chosen) |
| Height | GNSS / barometer | TF03 LiDAR |
| Yaw | GNSS + compass | compass, checked against the last GNSS heading |
| Absolute correction | GNSS | UWB anchors when the drone is near the entrance |

**Status: planned.** Open items:

- The non-GNSS sensor chain for Drone A is not designed. Optical flow and visual odometry from 30–50 m over rubble, smoke and dust are unproven with these parts.
- ArduPilot warns that switching back from a non-GPS source to GPS can make the position jump, and that non-GPS navigation suits slow vehicles. The switching rule needs a hold-and-blend procedure and testing.
- Drone A's hazard map is only as good as its position. Without GNSS, its map frame must still be tied to the UWB anchors so that Drone A's hazards and Drone B's survivor pins share one frame.

## Joining the two drones' maps

Drone A works in latitude and longitude; Drone B works in metres from the entrance. Two UWB anchors are surveyed with GNSS at setup, which fixes both the origin and the rotation between the frames. The rotation is the sensitive part: 5° of error puts a pin 30 m inside the structure 2.6 m off.

## What would prove real GPS independence

1. A real IWR6843 on the bench running the Doppler ego-velocity estimator, with its bias measured at rest and in motion. This is the single biggest credibility step.
2. The one-filter estimator in simulation, repeating the hover gate and the lawnmower search.
3. The radar vs optical-flow check flagging an injected bias, and the time it takes.
4. A UWB reset demonstrated in simulation, then on the bench.
5. Drone B hardware in the loop, then a first indoor flight.

## References

- ArduPilot. *GPS / Non-GPS Transitions.* https://ardupilot.org/copter/docs/common-non-gps-to-gps.html
- ArduPilot. *EKF Source Selection and Switching.* https://ardupilot.org/copter/docs/common-ekf-sources.html
- Kramer, A. et al. (2020). Radar-Inertial Ego-Velocity Estimation for Visually Degraded Environments. IEEE ICRA.
- Doer, C. & Trommer, G. F. (2020). An EKF Based Approach to Radar Inertial Odometry. IEEE MFI.
