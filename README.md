# Responsive Character Motion
This repo contains my Master's thesis project <b>Responsive Character Motion</b> <i>(June 1st, 2021)</i>, including motion matching, robot control and reinforcement learning.

There are 4 sub-projects included in this repo:
- <b>lab</b> contains a series of JavaScript files that helps understand theory applied in the project.
- <b>CartPolePD</b> is a test of trianing a PPO agent on a PD controller to perform the simple task of keeping a pole from collapsing while the cart is moving at a random velocity.
- <b>CharacterMotion</b> contains motion matching, feature extraction, inertialization blending and trajectory drawing.
- <b>CharacterMotionEnv</b> is wrapped as a Gym environment, it contains the trianing and testing of PPO on a physics based character, trying to mimick an animated, kinematic character.

<h2>Installation</h2>

<h2>Inputs in CharacterMotionEnv</h2>
<b>Moving camera:</b> <br/>
<b>Drawing path:</b> <br/>
<b>Movement settings:</b> <br/>

<h2>Training and testing in CharacterMotionEnv</h2>
<b>Training a model:</b>
<b>Testing the model:</b>

<h3>Abstract</h3>
Since the development of the Proximal Policy Optimization algorithm, researchers have utilized it for robot control, teaching a reinforcement learning agent how to retain balance of an animated, physics based character. This can be extended to games, enabling physics simulation of game characters, but the development introduces complexities that are not intuitively apparent in the research papers. This project dives into the complications of developing such a system, including motion matching, robot control and reinforcement learning. The motion matching system is developed and combined with inertialization blending for a smooth transition. The Proximal Policy Optimization algorithm is set up and trained on a physics based character that can then mimic the motion matched character, but due to the many hyperparameters, training proves to be more complicated that one might think.
