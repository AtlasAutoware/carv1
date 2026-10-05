# Long training runs on unicron: sharded, restartable after SIGKILL

unicron.csl.tjhsst.edu is the CSL's GPU node (96 threads, 440 GB RAM, 6x Quadro RTX 8000,
Ubuntu 24.04; also the Slurm `gpu` partition). From the laptop: `ssh unicron` (an entry in
~/.ssh/config jumps through ras2; the CSL password is asked twice, then the connection is
reused for 12 h).

## Layout on unicron (local disk)

    /var/tmp/2027eiyer/atlas/venv   python env, a copy of ~/atlas_venv (torch 2.14 cu130, onnxruntime 1.29)
    /var/tmp/2027eiyer/atlas/run    the run: code (ml/, f1tenth_gym_ros/, maps/, tools/), data/, runs_ext/, logs/shard/
    /var/tmp/2027eiyer/atlas/home   HOME for the jobs

Everything is on local disk because the home directory is NFS with Kerberos (sec=krb5i): a
process without a ticket (anything systemd starts, or anything still running after the ticket
expires) can neither read nor write it. Copy results back to the home or the laptop by hand.

## Run, check, stop

    R=/var/tmp/2027eiyer/atlas/run
    python3 $R/ml/shard_run.py install --root $R --set py=/var/tmp/2027eiyer/atlas/venv/bin/python
    python3 $R/ml/shard_run.py status  --root $R
    python3 $R/ml/shard_run.py stop    --root $R     # training checkpoints first; `install` again resumes

Logs: `$R/logs/shard/orchestrator.log` and one log per shard. Results: `$R/runs_ext/route/choice.json`
and `student.onnx` (the seed chosen on the selection set), per seed `$R/runs_ext/route_s<seed>/`.

## What makes it survive a SIGKILL (ml/shard_run.py)

- The run is 83 shards: 48 collect shards (50 expert episodes each), 3 merges, the data cache,
  5 training seeds (one GPU each, in parallel), 25 evals, the seed selection.
- A shard is done when its output file exists; every output is written to a temp file and renamed.
- Killed shards restart and resume: episodes are journaled one by one, training checkpoints every
  5 minutes and after every epoch (model, optimiser, LR schedule, RNG, position in the epoch).
- Every running shard holds a lock file, so a new orchestrator adopts shards a killed one left
  running instead of starting them twice.
- The orchestrator runs as a systemd user service (`Restart=on-failure`, back 15 s after a SIGKILL)
  with a timer every 2 minutes. This needs `loginctl enable-linger` (enabled Oct 5, 2026; undo with
  `loginctl disable-linger`). The units live in /run/user/<uid>/systemd/user because ~/.config is
  on the Kerberos NFS home. Crontab entries never ran for this account on unicron.
- A shard that keeps getting killed is restarted with growing delays (10 s doubling to 10 min).
- Not covered: a reboot, or killing every process of the account including the systemd user
  manager. Nothing is lost; run `install` again.

`python3 ml/unicron/kill_test.py <scratch copy of the repo> <venv python>` runs the whole pipeline
on a tiny config while killing the orchestrator, a collect shard, a training run mid-epoch and then
everything at once, and checks every output (ml/unicron/verify_run.py). Passed on Oct 5, 2026.

## Car-conditions run (car_run_1005.json)

The route-hint models above were trained and judged in a sim that differs from car 1 in ways
that matter (ml/car_model.py has the list and where each number comes from): the car's laser is
0.27 m and its camera 0.30 m ahead of the rear axle (the sim had both at the axle), ackermann_to_vesc
runs erpm mode (no speed between 0 and 0.78 m/s; /drive 1.0 = 1.03 m/s), the motor lags (60 ms,
~2 m/s^2 ramp), the right steering lock is 0.25 rad until the servo horn is re-centred, the lidar
drops ~19% of its bins, the route hint comes from a noisy, lagged SLAM pose and is replanned every
second as policy_bridge does, and a collision is any
part of the body, not the rear axle point. `sim_rollout.py --car <preset>` simulates all of that
(presets: car = today's car and bridge; car_bridge = with the 10/5 policy_bridge fixes; car_fixed =
also the horn re-centred; car_train / car_dagger = data collection).

car_run_1005.json: 2,400 expert demos under car conditions (same task sets as the route run,
expert top speed 1.0 m/s, a little steering noise), lidar-only (3 seeds) and camera+lidar with
camera augmentation and dropout (2 seeds); the five route-run seeds evaluated alongside as
`old_s*`; selection among the lidar-only models on car_sel (task seed 3000, car_bridge); then one
DAgger round (1,200 episodes driven by the chosen model, beta 0.5, the car as it is) and five more
lidar-only seeds on demos + DAgger; the final pick (`runs_car/final`) is over both rounds.
Every model gets the same evals: car_sel, car_std (car_bridge), car_today, car_fixed, car_cam (camera
perturbed), car_nocam (blank camera), car_instr (the bridge's default instruction), car_stress (double
latency and pose noise, 30% empty bins, 10% weaker steering) and car_aeb60 (AEB at 0.6 m instead of 0.35).

    R=/var/tmp/2027eiyer/atlas/car1     # code copied in; maps/ and data/_tasks/ copied from ../run (same task lists)
    cp ml/unicron/car_run_1005.json $R/shard_state/config.json
    python3 $R/ml/shard_run.py install --root $R
