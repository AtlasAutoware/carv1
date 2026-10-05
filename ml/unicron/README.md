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
