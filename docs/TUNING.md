# PID Tuning Workflow

## The loop

```
1. edit gains          PID_KP / PID_KI / PID_KD at the top of src/main.c
2. pio run -t upload
3. unplug              press the button, let the robot run
4. press again         stops the run and commits the log to flash
5. plug back in
6. python tools/telemetry.py --plot
```

The gains are written into each log's header, so every plot is titled with the
values that produced it. That is the whole point: after twenty runs, unlabelled
graphs are worthless.

A run also ends by itself when the buffer fills (~29 s) or at the 3-minute match
timeout. Either way the log is committed before the robot goes idle.

## First bring-up, before any tuning

Flash and watch the serial monitor. You want to see:

```
I (xxx) sumo: ToF sensors: left ok, right ok
I (xxx) sumo: LSM6DS3 ready, data-ready on GPIO 4
I (xxx) telemetry: ready: 3000 records, 96000 B RAM, partition at 0x190000
```

If the IMU line reports failure, work through this order:

| Symptom | Check |
|---|---|
| `LSM6DS3 not found at 0x6A` | `CS` tied to 3V3. A floating or low `CS` puts the part in SPI mode where it never answers on I²C. This is the most common cause by far. |
| still not found | `SA0` — if it is tied high the part is at `0x6B`, so pass `LSM6DS3_ADDR_HIGH` in `app_main`. |
| still not found | Bus scan. If the ToF sensors also vanish, the pull-ups are wrong (see docs/HARDWARE.md §6). |
| `INT1 setup failed` | `INT1` wired to GPIO 4. |

Then check the sign convention before trusting anything: rotate the robot
counter-clockwise by hand and confirm the logged heading goes **positive**. If it
goes negative, flip `IMU_YAW_SIGN` in `include/imu.h`. Getting this wrong makes
the controller drive the robot away from the target, and it looks exactly like
badly tuned gains.

## Order of tuning

Start where the code already is: `KP` only, `KI` and `KD` at zero.

1. **Kp** — raise until the robot just begins to oscillate about the target, then
   halve it. Too low and it turns sluggishly and loses the opponent; too high and
   it hunts.
2. **Kd** — bring in to damp the remaining overshoot. Because D is taken from the
   gyro's measured rate rather than from differentiating the heading, it is far
   less noisy than a textbook implementation and can usually be pushed harder
   than you would expect.
3. **Ki** — last, and sparingly. A sumo robot spends much of its time with the
   motors saturated, and integral action mostly buys windup there. The controller
   freezes the integrator while saturated, but the cleanest fix is still a small
   `KI`, or none.

## Reading the plots

Five panels, top to bottom:

- **Desired vs actual heading** — the money plot. Green shading marks where the
  opponent was actually in view; every setpoint step should sit inside a shaded
  region.
- **Error** — should decay without ringing.
- **PID contributions** — P, I and D plotted *separately*. When the robot
  oscillates, this panel says which term is responsible. If D is large and
  jagged, suspect mechanical vibration reaching the IMU before you touch the
  gains.
- **Motor outputs** — dotted lines at ±255. Long periods pinned to the rail mean
  the controller has no authority left and the gains above that point are
  meaningless.
- **Forward acceleration** — red shading marks flagged contact: commanded drive,
  but no acceleration and no rotation. That is the robot pushing something.

The summary line also prints sample-interval statistics. **Mean should be ~9.6 ms
with a small standard deviation.** If the spread is wide, the data-ready clock is
being disturbed and the D term is suspect — fix that before trusting any tuning
done from the run.

## Useful invocations

```bash
python tools/telemetry.py --plot                  # show it
python tools/telemetry.py --save run.png          # save instead
python tools/telemetry.py --csv run.csv           # export for Excel/MATLAB
python tools/telemetry.py --port COM5 --plot      # explicit port
```

Close the PlatformIO serial monitor first — esptool needs the port to itself.

Requires `numpy` and `matplotlib`; esptool ships with PlatformIO.

## Notes

- **Log data is not overwritten until the next run starts.** Flash is erased in
  `telemetry_begin_run`, i.e. on the button press, not at boot. Reflashing the
  firmware does not touch the telemetry partition — a run log survives until you
  deliberately start another run.
- Only the most recent run is kept. Export to CSV or PNG before the next run if
  you want to compare.
- `dropped` in the summary means the run outlasted the buffer; the log holds the
  first ~29 s. For step-response tuning that is the part that matters.
