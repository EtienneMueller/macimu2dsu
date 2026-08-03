# macimu2dsu

**MacBook IMU → DSU/CemuHook bridge**

Use the built-in accelerometer and gyroscope of an Apple Silicon MacBook as a motion controller for Cemu: no extra hardware, no phone, no Wii U GamePad.

Exposes the MacBook IMU as a [DSU/CemuHook](https://github.com/v1993/cemuhook-protocol) server on `127.0.0.1:26760`. Useful for _Breath of the Wild_ bow aiming and the motion-control shrines.

## Requirements

- Apple Silicon MacBook (tested on M1 Max)
- Cemu (tested with the Apple Silicon native fork [Cemu 2.6.2](https://github.com/neebyA/Cemu))
- [uv](https://github.com/astral-sh/uv)

Sensor access is provided by [macimu](https://github.com/olvvier/apple-silicon-accelerometer), which does the actual IOKit HID work. This project only maps its output onto the DSU protocol.

## Install

```bash
git clone https://github.com/EtienneMueller/macimu2dsu.git
cd macimu2dsu
uv sync
```

## Run

```bash
sudo uv run python3 macimu2dsu.py
```

Root is required because reading the IMU goes through IOKit HID.

Then in Cemu: Options → GamePad motion source → DSU1, server `127.0.0.1`, port `26760`.

## Usage

|Key|Action|
|---|---|
|`c`|Recalibrate gyro bias (hold still)|
|`d`|Toggle raw output|
|`r`|Force settings reload|
|`q`|Quit|

## Settings

`settings.json` is reloaded live: edit it while the server runs and the change takes effect immediately.

|Key|Meaning|
|---|---|
|`accel_map`|Which sensor axis feeds each DSU accel axis|
|`gyro_map`|Which sensor axis feeds each DSU gyro axis|
|`accel_scale` / `gyro_scale`|Output multipliers|
|`gyro_deadzone`|deg/s below which gyro output is clamped to zero|

Axis maps use `x`/`y`/`z` for the sensor's own axes, `ax`/`ay`/`az` and `gx`/`gy`/`gz` to pull from the other sensor, `0` to zero an axis, and a `-` prefix to negate.

The accel axes are zeroed by default, so orientation comes from gyro integration alone. Startup calibration plus a small deadzone keeps the drift manageable. Press `c` to re-calibrate if it creeps during a long session.

## Known limitations

- The default axis map is calibrated for my MacBook model. Other models may mount the IMU differently. If the axes are wrong for yours, press `d` to watch the values and edit `settings.json`. PRs with working maps for other models are welcome.
- No absolute tilt reference, so orientation drifts slowly over time.
- Motion only. Buttons and sticks still come from your keyboard or gamepad.

## Credits

- [macimu](https://github.com/olvvier/apple-silicon-accelerometer) by [@olvvier](https://github.com/olvvier): Apple Silicon IMU access, which this project is built on
- [cemuhook-protocol](https://github.com/v1993/cemuhook-protocol): DSU protocol specification

## License

MIT