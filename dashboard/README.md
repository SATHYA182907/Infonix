# IF-01 Thermal Optimizer dashboard

This is the HMI dashboard for *Intelligent Thermal Optimization and Predictive Control for Induction Furnaces* (The Minda Project, Sakthi Hackathon 1.0).

To use it, open `index.html` in a browser. It is a single file with no build step.

What it shows:
- **Heat setup**: the operator enters the metal grade, charge weight, initial temperature and target temperature.
- **Energy prediction**: a physics model (m·cp·ΔT + m·L + losses) plus an AI correction learned from past heats.
- **MPC recommendation**: power setpoint, ramp rate, stage-transition temperature and hold time, picked by searching candidate trajectories for the lowest energy and time cost.
- **Operator approval → PLC interlocks → actuation**: closed-loop control, with an emergency stop.
- **Live melt trend**: the actual run against the AI forecast, the target and the manual baseline, with a re-check every 5 minutes.
- **Savings**: a baseline vs optimized comparison, the deck's prototype results, the AI learning scatter, and a yearly plant impact projection.

The furnace is simulated with a lumped-enthalpy thermal model. To run on real hardware, replace `makeSim` and the live `tick` loop with the Raspberry Pi sensor feed.
`furnace-hmi.html` is the same page without the document wrapper (artifact format).
