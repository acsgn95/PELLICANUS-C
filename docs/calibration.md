# Calibration

Two tools in `tools/` calibrate the board's sensors without equipment, by hand on a table. Each one
guides a session with the board connected over USB, keeps the recording, and writes the result as JSON.
They need Python 3 with numpy and scipy (`pyserial` is optional).

Close any other reader of the port first (`pellicanus_reader.py`): only one program can read it.

## IMU: `calibrate_imu.py`

```
python3 tools/calibrate_imu.py [--port /dev/ttyACM0] [--location LAT,LON,HEIGHT] [--save PREFIX]
```

1. It asks for the location: latitude and longitude in degrees, height above sea level in metres.
   Local gravity, the reference for the accelerometer, is WGS84 normal gravity at that latitude and
   height. It ranges from 9.780 m/s² at the equator to 9.832 m/s² at the poles, a 0.5 % spread that
   would otherwise end up in the scale factors.
2. It asks for 18 poses, one after another:
   - the first one, flat with the IMU's +Z axis up, is held for 10 s; it sets the gyro bias and the
     noise level that stillness is judged against;
   - then the six faces;
   - then the twelve edges, tilted half way (rest the board against a book).

   For each pose, place the board, let go of it, and press Enter. While you place it, the tool shows
   how far the board is from the pose and whether it is still. The pose is accepted after 3 s of
   stillness within 25° of the target.
3. Move the board between poses by hand. These turns are part of the calibration: the gyro is
   calibrated from them.

The method is Tedaldi, Pretto and Menegatti, "A robust and easy to implement method for IMU
calibration without external equipments" (ICRA 2014).

- **Accelerometer.** In every still pose, the corrected reading must have the magnitude of local
  gravity. This gives three scale factors, three misalignment terms and three biases.
- **Gyro.** The rotation it integrates between two still poses must carry the first pose's gravity
  direction onto the second's. This gives three scale factors and six misalignment terms. The bias
  comes from the first pose.

The models below are applied to the sensor's output, in its own axes and units:

```
accel_cal = accel_matrix @ (accel_raw + accel_bias)      m/s²
gyro_cal  = gyro_matrix  @ (gyro_raw  - gyro_bias)       rad/s
```

The tool prints the accelerometer's magnitude error over the poses and the gyro's error in carrying
gravity, before and after calibration. On a good session, these are below about 1 mg and a few tenths
of a degree.

## Magnetometer: `calibrate_mag.py`

```
python3 tools/calibrate_mag.py [--port /dev/ttyACM0] [--field-gauss B] [--imu-calibration FILE.json] [--save PREFIX]
```

Run it away from steel, magnets and computers; a wooden or plastic table outdoors is best. Everything
on the board stays as it will fly: its hard and soft iron are what is being measured. A session has
three parts:

1. **Rest.** The board rests flat for 10 s.
2. **Turning.** Turn the board slowly through every direction: roll it, flip it, point each face
   down in turn. A bar shows the share of the sphere of field directions covered so far. Press Enter
   once it is past 80 %.
3. **Still poses.** The board rests in eight poses, each at a new heading.

From the session, the tool computes:

- **Hard and soft iron.** An ellipsoid fitted to all samples, refined so that the corrected field has
  a constant magnitude:

  ```
  mag_cal = soft_iron @ (mag_raw - hard_iron)      gauss, |mag_cal| = B
  ```

  B is `--field-gauss` when it is given, else the mean radius of the fit, which keeps the sensor's
  own scale. Take B from a world magnetic model at the location, for example NOAA's calculator. B
  matters only if the field's magnitude is used. Directions and headings do not depend on it.
- **Rotation from the magnetometer's axes to the IMU's.** In every still pose, the angle between
  gravity and the field (the dip) is the same. The rotation that keeps it constant over the poses is
  the one between the two chips. It is better with the IMU's calibration
  (`--imu-calibration PREFIX_imu_calibration.json`).

## Files

- `PREFIX` defaults to `calibration/imu_<date>_<time>` or `calibration/mag_<date>_<time>`.
- A session writes `PREFIX_imu.csv` and `PREFIX_mag.csv`, in `pellicanus_reader.py --save`'s format.
  The result goes to `PREFIX_imu_calibration.json` or `PREFIX_mag_calibration.json`.
- Calibrate a recording again, for example with other settings, with `--from PREFIX`:

  ```
  python3 tools/calibrate_imu.py --from PREFIX [--location LAT,LON,HEIGHT | --gravity G]
  python3 tools/calibrate_mag.py --from PREFIX [--field-gauss B] [--imu-calibration FILE.json]
  ```

  A recording made with `pellicanus_reader.py --save` works as well, if it starts with 10 s at rest
  and holds still poses as above.
- `tools/test_calibration_synthetic.py` checks both tools on a synthetic recording with known errors.

Calibration depends on temperature. The result records the temperature of the session; calibrate
near the temperature the board will work at.
