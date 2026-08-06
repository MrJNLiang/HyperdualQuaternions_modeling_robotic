#!/bin/bash

python3 experiments/run_grasp_circle.py --mode load --gains tuned --condition none
python3 experiments/run_grasp_circle.py --mode load --law dq-ctc --condition none
python3 experiments/run_grasp_circle.py --mode load --law dq-chandra --condition none
python3 experiments/run_grasp_circle.py --mode load --law dq-hinf --condition none
python3 experiments/run_grasp_circle.py --mode load --gains tuned --condition highspeed
python3 experiments/run_grasp_circle.py --mode load --law dq-ctc --condition highspeed
python3 experiments/run_grasp_circle.py --mode load --law dq-chandra --condition highspeed
python3 experiments/run_grasp_circle.py --mode load --law dq-hinf --condition highspeed
python3 experiments/run_grasp_circle.py --mode load --gains tuned --condition fast-transit
python3 experiments/run_grasp_circle.py --mode load --law dq-ctc --condition fast-transit
python3 experiments/run_grasp_circle.py --mode load --law dq-chandra --condition fast-transit
python3 experiments/run_grasp_circle.py --mode load --law dq-hinf --condition fast-transit
python3 experiments/run_grasp_circle.py --mode load --gains tuned --condition noise
python3 experiments/run_grasp_circle.py --mode load --law dq-ctc --condition noise
python3 experiments/run_grasp_circle.py --mode load --law dq-chandra --condition noise
python3 experiments/run_grasp_circle.py --mode load --law dq-hinf --condition noise 
python3 experiments/run_grasp_circle.py --mode load --gains tuned --condition coarse-dt
python3 experiments/run_grasp_circle.py --mode load --law dq-ctc --condition coarse-dt
python3 experiments/run_grasp_circle.py --mode load --law dq-chandra --condition coarse-dt
python3 experiments/run_grasp_circle.py --mode load --law dq-hinf --condition coarse-dt

python3 experiments/run_grasp_circle.py --mode noload --gains tuned --condition none
python3 experiments/run_grasp_circle.py --mode noload --law dq-ctc --condition none
python3 experiments/run_grasp_circle.py --mode noload --law dq-chandra --condition none
python3 experiments/run_grasp_circle.py --mode noload --law dq-hinf --condition none
python3 experiments/run_grasp_circle.py --mode noload --gains tuned --condition highspeed
python3 experiments/run_grasp_circle.py --mode noload --law dq-ctc --condition highspeed
python3 experiments/run_grasp_circle.py --mode noload --law dq-chandra --condition highspeed
python3 experiments/run_grasp_circle.py --mode noload --law dq-hinf --condition highspeed
python3 experiments/run_grasp_circle.py --mode noload --gains tuned --condition fast-transit
python3 experiments/run_grasp_circle.py --mode noload --law dq-ctc --condition fast-transit
python3 experiments/run_grasp_circle.py --mode noload --law dq-chandra --condition fast-transit
python3 experiments/run_grasp_circle.py --mode noload --law dq-hinf --condition fast-transit
python3 experiments/run_grasp_circle.py --mode noload --gains tuned --condition noise
python3 experiments/run_grasp_circle.py --mode noload --law dq-ctc --condition noise
python3 experiments/run_grasp_circle.py --mode noload --law dq-chandra --condition noise
python3 experiments/run_grasp_circle.py --mode noload --law dq-hinf --condition noise 
python3 experiments/run_grasp_circle.py --mode noload --gains tuned --condition coarse-dt
python3 experiments/run_grasp_circle.py --mode noload --law dq-ctc --condition coarse-dt
python3 experiments/run_grasp_circle.py --mode noload --law dq-chandra --condition coarse-dt
python3 experiments/run_grasp_circle.py --mode noload --law dq-hinf --condition coarse-dt

python3 experiments/run_grasp_circle.py --compare-only --plot

python3 experiments/plot_grasp_results.py --mode load
python3 experiments/plot_grasp_results.py --mode noload

