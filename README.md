# Autonomous Rendezvous Decision Transformer
This repository contains the implementation code for a Decision Transformer applied to a Close-Range rendezvous maneuver. The goal of this work is the generation of a database of high-quality rendezvous via an optimal control approach, and the training of a [Decision Transformer](https://github.com/kzl/decision-transformer) based on the GPT2 architecture.
## Rendezvous modeling
The environment comprises 2 spacecraft, a chaser, and an uncontrolled target in tumbling motion. The objective of the chaser is to position itself from its initial state to a known berthing point in the tumbler relative frame with a velocity and rotation compatible with the tumbling motion of the target. The dynamics are defined by the Clohessy-Wiltshire equations.

![Image from George Boyarko et al. "Optimal Rendezvous Trajectories of a Controlled Spacecraft and a Tumbling Object"](https://github.com/Govax99/autonomous-rendezvous-decision-transformer/blob/main/images/model.png)
<sub>\*Image from George Boyarko et al. "Optimal Rendezvous Trajectories of a Controlled Spacecraft and a Tumbling Object"</sub>

## Database generation
The first part of the thesis consists of the generation of successful rendezvous. The agent's behavior quality will be as good as the data on which he will be trained. The [Yop](https://www.yoptimization.com/) optimal control direct solver is used for trajectory generation, starting from initial guesses obtained by PD control. The objective is to minimize the L2 norm of the control action, while a keep-out zone around the target is the main constraint on the chaser motion.

## Decision transformer
The agent is taken as the [Hugging Face](https://huggingface.co/docs/transformers/model_doc/decision_transformer) implementation of the Decision Transformer, based on a GPT2 architecture. The training was done using the following hyperparameters TODO

## How to use
To generate additional trajectories, follow these steps:

 - Clone this GitHub repository
 - In the optimal-control folder download and unzip [casadi](https://github.com/casadi/casadi/releases/download/3.6.3/casadi-3.6.3-linux64-matlab2018b.zip)
 - In the optimal-control folder download and unzip [Yop](https://github.com/yoptimization/yop/archive/refs/tags/v1.0-rc3.zip)
 - From the shell: `matlab -r "create_database -ntot N -nsave M -printlevel 2 -savedir results -rngseed 42"` (check create_database doc for more info on the command arguments)

## Update
As of today (22 December 2023) the project is frozen, after unsatisfactory results with the first database batch.
