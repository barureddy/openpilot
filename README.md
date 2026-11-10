<div align="center" style="text-align: center;">

<h1>openpilot</h1>

<p>
  <b>openpilot is an operating system for robotics.</b>
  <br>
  Currently, it upgrades the driver assistance system in 300+ supported cars.
</p>

<h3>
  <a href="https://docs.comma.ai">Docs</a>
  <span> · </span>
  <a href="https://docs.comma.ai/contributing/roadmap/">Roadmap</a>
  <span> · </span>
  <a href="https://github.com/commaai/openpilot/blob/master/docs/CONTRIBUTING.md">Contribute</a>
  <span> · </span>
  <a href="https://discord.comma.ai">Community</a>
  <span> · </span>
  <a href="https://comma.ai/shop">Try it on a comma four</a>
</h3>

Quick start: `bash <(curl -fsSL openpilot.comma.ai)`

[![openpilot tests](https://github.com/commaai/openpilot/actions/workflows/tests.yaml/badge.svg)](https://github.com/commaai/openpilot/actions/workflows/tests.yaml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![X Follow](https://img.shields.io/twitter/follow/comma_ai)](https://x.com/comma_ai)
[![Discord](https://img.shields.io/discord/469524606043160576)](https://discord.comma.ai)

</div>

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://github.com/commaai/openpilot/assets/8762862/2f7112ae-f748-4f39-b617-fabd689c3772"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://github.com/commaai/openpilot/assets/8762862/92351544-2833-40d7-9e0b-7ef7ae37ec4c"></a></td>
    <td><a href="https://youtu.be/SUIZYzxtMQs" title="A drive to Taco Bell"><img src="https://github.com/commaai/openpilot/assets/8762862/05ceefc5-2628-439c-a9b2-89ce77dc6f63"></a></td>
  </tr>
</table>


jvePilot Hybrid ACC for Jeep Cherokee (KL / CUSW) & FCA Vehicles
------

This branch ports **jvePilot**’s Hybrid Adaptive Cruise Control (ACC) longitudinal speed-control suite to modern `openpilot` and `opendbc`, with full support for **CUSW** vehicles (2014–2018 Jeep Cherokee KL using `CRUISE_BUTTONS` `0x2FA`) as well as non-CUSW FCA vehicles (Chrysler Pacifica, Jeep Grand Cherokee, Ram using `CRUISE_BUTTONS` `0x23B`).

Because FCA vehicles with factory DASM/ACC keep the stock radar in control of gas and brake actuators, **jvePilot** bridges openpilot's longitudinal planner (`modelV2` vision + lead tracking) with the vehicle's stock ACC by transmitting virtual steering-wheel cruise button presses (`ACC_Accel` / `Cruise +`, `ACC_Decel` / `Cruise -`, `ACC_Resume`, `ACC_Cancel`, `ACC_Distance_Inc`, `ACC_Distance_Dec`) over the powertrain CAN bus.

### Features Included

1. **Smoother Driving in Traffic**: Uses openpilot's longitudinal MPC (`LongitudinalPlanner`) to predict lead vehicle motion and smoothly modulate the vehicle's ACC set speed (`CC.actuators.speed`) via virtual `ACC_Accel` / `ACC_Decel` button pulses (`20 ms` hold on an `80 ms` / `12.5 Hz` cycle with a `0.35 m/s` deadband).
2. **Proactive ACC Braking**: When openpilot's longitudinal plan calls for deceleration (`aTarget < -0.1 m/s²`), `controlsd` sets the target ACC speed below current vehicle speed (`vEgo + aTarget * 2.5`) so the stock ACC module engages brakes sooner instead of coasting.
3. **Slow for Cars Cutting In Before Stock ACC Does**: Uses openpilot's camera-based vision lead detection (`modelV2.leadsV3` in `radard`) combined with proactive ACC braking to lower the ACC set speed as soon as a cut-in vehicle is seen by the camera—before the narrow-beam radar locks on.
4. **Slow in a Turn (Configurable)**: Computes lookahead road curvature from `modelV2.orientationRate.z` and `modelV2.velocity.x` (`get_turn_speed_limit`) and automatically lowers the ACC set speed before and through curves, then restores your set speed exiting the turn.
5. **0 mph Stop-and-Go Auto Resume (Configurable)**: Automatically sends `ACC_Resume` when the vehicle comes to a complete stop behind a lead vehicle and traffic begins moving (`!longitudinalPlan.shouldStop`). Also fixes `ret.cruiseState.standstill` detection on CUSW (`CarState.update_cusw`).
6. **Auto Follow Distance (Configurable)**: Dynamically adjusts both openpilot's MPC follow time gap and the vehicle's hardware ACC follow-distance bars (`1–4 bars` via virtual `ACC_Distance_Inc` / `ACC_Distance_Dec` presses) based on vehicle speed. Pressing the physical steering-wheel distance button toggles manual distance override for the current engagement.
7. **ACC Eco Throttle Limiting (Configurable)**: Caps how far above current speed (`vEgo`) openpilot pushes the ACC set speed when accelerating back up to cruise speed, preventing aggressive downshifts and wide-open throttle surges.
8. **Gas Pedal Does Not Disengage (Configurable)**: Pressing the accelerator pedal overrides speed without disengaging openpilot (`DisengageOnAccelerator` defaults to `Off`). Pressing `SET-` while holding the gas pedal syncs `v_cruise` to your current speed.
9. **Speedometer Display Sync**: Syncs openpilot's on-road speed readout (`vEgoCluster`) with the vehicle's instrument cluster speedometer (`CLUSTER_1["SPEEDOMETER"]` on CUSW / `ESP_8["Vehicle_Speed"]` on Pacifica/Grand Cherokee).
10. **Gas / Brake Speed Indicator Colors**: Colors the on-screen current speed readout **green** when accelerating (`aEgo > 0.15 m/s²` or gas pressed) and **red** when braking (`aEgo < -0.15 m/s²` or brake pressed) while engaged.

---

### Settings & Configuration (`Settings → Toggles`)

All `jvePilot` settings are located in **Settings → Toggles** on the device UI, persist across reboots, and update live while driving without requiring a restart:

| Setting Name (`Params` Key) | Options / Type | Default | Description & Behavior |
|---|---|---|---|
| **jvePilot ACC Speed Control** (`JvePilotAccControl`) | `Off` / `On` (Toggle) | **`Off`** (Stock ACC) | Master switch between **Stock Adaptive Cruise Control** (`Off`) and **jvePilot Hybrid ACC Speed Control** (`On`). When `Off`, openpilot sends zero virtual speed or distance button presses and the vehicle's factory ACC manages set speed directly. |
| **jvePilot Auto Resume from Stop** (`JvePilotAutoResume`) | `Off` / `On` (Toggle) | **`On`** | When enabled, automatically sends `ACC_Resume` when stopped at 0 mph behind a lead car as soon as traffic moves forward. When disabled, you must tap `RES+` or the gas pedal to resume from a standstill. |
| **jvePilot Slow in Turns** (`JvePilotSlowInTurns`) | `Off`, `Relaxed`, `Standard`, `Aggressive` | **`Standard`** | Controls how much openpilot reduces ACC set speed in curves via lateral acceleration limits:<br>• **Off**: No curve speed reduction<br>• **Relaxed**: `1.6 m/s²` max lateral accel (slows more in turns)<br>• **Standard**: `2.0 m/s²` max lateral accel<br>• **Aggressive**: `2.4 m/s²` max lateral accel (takes turns faster) |
| **jvePilot Auto Follow** (`JvePilotAutoFollow`) | `Off`, `Relaxed`, `Standard`, `Aggressive` | **`Standard`** | Automatically adjusts ACC follow-distance bars (`1 → 2 → 3 → 4 bars`) and MPC follow gap as speed increases (with `~1.8 mph` hysteresis):<br>• **Off**: Manual follow distance only<br>• **Relaxed**: Steps bars at `12 / 25 / 42 mph` (longer distance sooner)<br>• **Standard**: Steps bars at `15 / 30 / 50 mph`<br>• **Aggressive**: Steps bars at `18 / 36 / 58 mph` (closer following) |
| **jvePilot ACC Eco** (`JvePilotAccEco`) | `Off`, `Eco 1`, `Eco 2` | **`Eco 1`** | Limits how far above `vEgo` the ACC target speed is set during acceleration:<br>• **Off**: Up to `+5.0 m/s` (`~11 mph`) lead<br>• **Eco 1**: Up to `+3.0 m/s` (`~6.7 mph`) lead<br>• **Eco 2**: Up to `+1.8 m/s` (`~4.0 mph`) lead (gentlest throttle) |
| **Disengage on Accelerator Pedal** (`DisengageOnAccelerator`) | `Off` / `On` (Toggle) | **`Off`** | When `Off` (default), pressing the gas pedal does not disengage openpilot. |
| **Driving Personality** (`LongitudinalPersonality`) | `Aggressive`, `Standard`, `Relaxed` | **`Standard`** | Unlocked when **jvePilot ACC Speed Control** is `On` (used when **jvePilot Auto Follow** is `Off`). |

---

### Comparison with Upstream `jvePilot` (`j-vanetten/openpilot`)

| Area | Upstream `jvePilot` (`j-vanetten/openpilot`) | This Branch (`barureddy/openpilot` — `jvePilot`) |
|---|---|---|
| **Supported FCA Platforms** | Non-CUSW FCA only (Pacifica, Grand Cherokee, Ram using `WHEEL_BUTTONS` `0x23B`, `DAS_3`, `DAS_4`, `ESP_8`). | Supports **both** **CUSW** (`CAR.JEEP_CHEROKEE_5TH_GEN` using `CRUISE_BUTTONS` `0x2FA`, `ACC_CONTROL`, `ACC_HUD`, `CLUSTER_1`) and non-CUSW FCA (`0x23B`). |
| **CUSW `CRUISE_BUTTONS` Counter & Standstill** | Not present (CUSW not supported in upstream `jvePilot`). | Fixed `self.button_counter` in `CarState.update_cusw` (was stuck at `0`, causing transmitted `0x2FA` frames to have a frozen counter of `1`) and added `ret.cruiseState.standstill` on CUSW. |
| **Panda Safety Layer** | Allows `CRUISE_BUTTONS` (`0x23B`) in Panda safety. | Enforces single-button-only TX safety in both `chrysler_cusw.h` (`0x2FA`) and `chrysler.h` (`0x23B`): allows `ACC_Resume`, `ACC_Accel`, `ACC_Decel`, `ACC_Distance_Inc`, `ACC_Distance_Dec` when `controls_allowed == true`, and `ACC_Cancel` always. |
| **Default Mode (Stock ACC vs jvePilot)** | `jvePilot` hybrid ACC is always active when engaged. | Configurable via **jvePilot ACC Speed Control** in `Settings → Toggles`, defaulting to **Off (Stock ACC)** per user preference. |
| **UI Controls for `ACC Eco` & `Auto Follow`** | Placed as custom Qt overlay buttons (leaf icon / auto-follow icon) on the legacy openpilot 0.8/0.9 Qt on-road screen. | Integrated natively into modern openpilot's Raylib **Settings → Toggles** menu (`multiple_button_item` selectors), plus physical steering-wheel distance button override on-road. |


Using openpilot in a car
------

To use openpilot in a car, you need four things:
1. **Supported Device:** a comma four, available at [comma.ai/shop/comma-four](https://www.comma.ai/shop/comma-four).
2. **Software:** The setup procedure for the comma four allows users to enter a URL for custom software. Use the URL `openpilot.comma.ai` to install the release version.
3. **Supported Car:** Ensure that you have one of [the 300+ supported cars](docs/CARS.md).
4. **Car Harness:** You will also need a [car harness](https://comma.ai/shop/car-harness) to connect your comma four to your car.

We have detailed instructions for [how to install the harness and device in a car](https://comma.ai/setup). Note that it's possible to run openpilot on [other hardware](https://blog.comma.ai/self-driving-car-for-free/), although it's not plug-and-play.


### Branches

Running `master` and other branches directly is supported, but it's recommended to run one of the following prebuilt branches:

| comma four branch      | comma 3X branch        | URL                                    | description                                                                         |
|------------------------|------------------------|----------------------------------------|-------------------------------------------------------------------------------------|
| `release-mici`         | `release-tizi`         | openpilot.comma.ai                     | This is openpilot's release branch.                                                 |
| `release-mici-staging` | `release-tizi-staging` | openpilot-test.comma.ai                | This is the staging branch for releases. Use it to get new releases slightly early. |
| `nightly`              | `nightly`              | openpilot-nightly.comma.ai             | This is the bleeding edge development branch. Do not expect this to be stable.      |
| `nightly-dev`          | `nightly-dev`          | installer.comma.ai/commaai/nightly-dev | Same as nightly, but includes experimental development features for some cars.      |

For [chestnut](https://comma.ai/shop/chestnut), use the following installer URLs:

| branch                       | URL                                                        | description                                                                         |
|------------------------------|------------------------------------------------------------|-------------------------------------------------------------------------------------|
| `release-chestnut`           | installer.comma.ai/commaai/release-chestnut                | This is openpilot's release branch.                                                 |
| `release-chestnut-staging`   | installer.comma.ai/commaai/release-chestnut-staging        | This is the staging branch for releases. Use it to get new releases slightly early. |
| `nightly-chestnut`           | installer.comma.ai/commaai/nightly-chestnut                | This is the bleeding edge development branch. Do not expect this to be stable.      |
| `nightly-chestnut-dev`       | installer.comma.ai/commaai/nightly-chestnut-dev            | Same as nightly, but includes experimental development features for some cars.      |

To start developing openpilot
------

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot).

* Join the [community Discord](https://discord.comma.ai)
* Check out [the contributing docs](docs/CONTRIBUTING.md)
* Check out the [openpilot tools](openpilot/tools/)
* Code documentation lives at https://docs.comma.ai
* Information about running openpilot lives on the [community wiki](https://github.com/commaai/openpilot/wiki)

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs#open-positions) and offers lots of [bounties](https://comma.ai/bounties) for external contributors.

Safety and Testing
----

* openpilot observes [ISO26262](https://en.wikipedia.org/wiki/ISO_26262) guidelines, see [SAFETY.md](docs/SAFETY.md) for more details.
* openpilot has software-in-the-loop [tests](.github/workflows/tests.yaml) that run on every commit.
* The code enforcing the safety model lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software-in-the-loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware-in-the-loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware-in-the-loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 comma devices continuously replaying routes.

<details>
<summary>MIT Licensed</summary>

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**
</details>

<details>
<summary>User Data and comma Account</summary>

By default, openpilot uploads driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software, and users can disable data collection if they wish.

openpilot logs the road-facing cameras, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera and microphone are only logged if you explicitly opt-in in settings.

By using openpilot, you agree to [our Privacy Policy](https://comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.
</details>
