# Uncertainty-Aware Decision-Making Framework for Autonomous Vehicles

This repository contains the simulation code and quantitative testing framework for research investigating uncertainty-aware decision-making in autonomous vehicles.

## Project Overview
Traditional autonomous controllers often rely solely on deterministic trajectory predictions, making them vulnerable to unexpected road behavior or degraded sensor inputs. This project introduces a lightweight, uncertainty-aware decision controller that dynamically calculates a unified risk score to transition the vehicle from standard driving to a proactive cautious mode before safety margins are compromised.

## Core Risk Formula
The controller evaluates real-time perception unreliability using the following weighted metric:

Risk Score (R) = (0.4 * U_epi) + (0.4 * U_ale) + (0.2 * E_pred)

- U_epi (Epistemic Uncertainty): Captures model limitations and internal confusion.
- U_ale (Aleatoric Uncertainty): Accounts for inherent noise and sensor variability.
- E_pred (Prediction Error): Measures the deviation between predicted and observed kinematic positions.

When the total risk score exceeds the 0.40 threshold, the system activates a cautious operational mode, widening buffer distances proactively.

## Simulation Scenarios
The simulation environment evaluates performance across five primary failure domains:
1. Pedestrian Crosswalks: Benchmarks system efficiency and checks for false positives during routine interactions.
2. Vehicle Cut-Ins: Evaluates proactive response speed and separation safety margins during abrupt lane changes.
3. Lead-Vehicle Braking: Tests system reaction to sudden deceleration of preceding traffic.
4. Unprotected Intersection: Assesses cautious decision-making in complex multi-path interactions with right-of-way ambiguity.
5. Environmental Degradation: Tests system resilience under high sensor noise and reduced visibility conditions.


## Running the Simulation
1. Ensure **Python 3.8** (or any compatible version) is installed along with the CARLA simulator.
2. Open your Command Prompt (CMD) or terminal in the project folder.
3. Run the main simulation application using:
   python iris_Carla_app.py
