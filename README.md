# Experiment Handler

ROS package for handling real robot experiment

Principle :

When making robots act in the real world using reinforcement learning, you need to give them some reward when they are in a set of states (and eventually do an action). While the task's inherent MDP is contained in the environment and the way the perceptual system build states, the reward still must be sent by some "god-like" system that monitor what happens.

If a complete robot architecture would self-generate reward (e.g. from a motivation system), in simpler architecture, we want to provide the learning feedback to our robot easily. This is the purpose of this ROS package.

It was initially designed to work with the PR2 robot, but any robot being controlled by discrete RL agent will fit.
