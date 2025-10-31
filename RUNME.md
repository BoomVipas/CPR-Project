# CPR Grid Simulation — Quick Run Guide

## Prerequisites

- Python 3.11 or newer (only standard-library modules are required)
- Project root: `/Users/vipas/Desktop/CPR Project`

## Run the simulation

```bash
python main.py --ticks 60 --seed 123 --gold 25 --print-every 1
```

## Optional animation

```bash
python main.py --ticks 60 --seed 123 --gold 25 --print-every 1 --sleep-sec 0.2 --animate
```

## Browser visualizer

1. Run a simulation (default settings already generate `frames.json` and `log.txt`):
```bash
python main.py --ticks 200 --print-every 20
```
2. Open `visualization.html` in your browser and press play to step through the saved frames. Refresh the page after each new run to load the latest data.
   - Use the `Robot inspector` dropdown to follow a specific robot; the sidebar updates with its Paxos state and timers as you scrub frames.
   - Want the CSP handshake from the milestone report instead of Paxos? Add `--consensus handshake` to switch strategies.

## Fault injection knobs

```bash
python main.py --ticks 120 --message-delay-min 1 --message-delay-max 3 --message-loss 0.05 --message-reorder 0.25
```

Use these flags to test Paxos retries and the new asynchronous message bus under lossy, out-of-order conditions.

## Discover more options

```bash
python main.py --help
```

## Run the rule tests

```bash
pytest tests/test_simulation_rules.py
```

These scenario tests cover pickup, carrying, dropping, and deposit rules for the robots.
