# Responsive Character Motion
This repo contains my Master's thesis project <b>Responsive Character Motion</b> <i>(June 1st, 2021)</i>, including motion matching, robot control and reinforcement learning.

There are 4 sub-projects included in this repo:
- <b>lab</b> contains a series of JavaScript files that helps understand theory applied in the project.
- <b>CartPolePD</b> is a test of trianing a PPO agent on a PD controller to perform the simple task of keeping a pole from collapsing while the cart is moving at a random velocity.
- <b>CharacterMotion</b> contains motion matching, feature extraction, inertialization blending and trajectory drawing.
- <b>CharacterMotionEnv</b> is wrapped as a Gym environment, it contains the trianing and testing of PPO on a physics based character, trying to mimick an animated, kinematic character.

<h2>Installation</h2>
To run the project, you will first need to install Python and a series of Python modules. Installing all of the modules will ensure that you are able to use run every included sub-project, the project is known to work on the specified version. Modules required:

>>python==3.7.0\
>>pip==21.0.1\
>>bvh==0.3\
>>gym==0.18.0\
>>mpi4py==3.0.3\
>>numpy==1.19.2\
>>optuna==2.7.0\
>>pillow==7.2.0\
>>pybullet==3.1.0\
>>tensorboard==2.4.0\
>>transformations==2020.1.1\
>>scipy==1.6.1\
>>pandas==1.1.5\
>>opencv-python==4.5.1.48\
>>stable-baselines[mpi]==2.10.1\
>>numpy-base==1.19.2\
>>kiwisolver==1.3.1\
>>matplotlib==3.3.4

<b>NOTE:</b> Some of the module versions are not known to pip, for a correct installation, please do as following:
1. Install Visual Studio 2019 C++ redistribution: One way to do this is by installing Visual Studio, and then installing the 'Desktop Development with C++' package using the Visual Studio Installer.
2. Download and install Anaconda: https://www.anaconda.com/products/individual#Downloads
3. Open Anaconda terminal and set up a new Anaconda environment to contain the modules specific for this project:
>>conda create -n RCM python=3.7 anaconda\
>>conda activate RCM
5. Install MPI & stable-baselines: Follow this guide on how to install MPI and stable-baselines 'https://stable-baselines.readthedocs.io/en/master/guide/install.html'
7. Install Conda-specific versions:
>>conda install matplotlib==3.3.4\
>>conda install numpy-base==1.19.2\
>>conda install pip==21.0.1
8. Install pip-specific versions:
>>pip install bvh==0.3\
>>pip install gym==0.18.0\
>>pip install mpi4py==3.0.3\
>>pip install numpy==1.19.2\
>>pip install optuna==2.7.0\
>>pip install pillow==7.2.0\
>>pip install pybullet==3.1.0\
>>pip install tensorboard==2.4.0\
>>pip install tensorflow==1.15.0\
>>pip install transformations==2020.1.1\
>>pip install scipy==1.6.1\
>>pip install pandas==1.1.5\
>>pip install opencv-python==4.5.1.48\
>>pip install kiwisolver==1.3.1
9. If everything was installed correct, you should be able to run the project by first CD'ing into the project location using an Anaconda terminal with the RCM environment active, and then writing 'python CharacterMotion/main.py', 'python CharacterMotionEnv/runBasic.py' or 'python CartPolePD/run.py'. If you are using either runBasic.py or run.py, you will need to first run server.py from the respective project folder in another Anaconda terminal, under the same environment. A model can be trained using the 'CharacterMotionEnv/trainBasic.py' file, but this is not suggested unless you understand what the file does.

<h2>Inputs in CharacterMotion</h2>
<b>Moving camera:</b> Hold down [alt] and left mouse-button down simultaneously, move mouse around. right mouse-button is zoom, middle mouse-button is pan.<br/>
<b>Drawing path:</b> Left-click with mouse to put down a single point, right-click to clear. Hold down left mouse-button to put down points continuously.<br/>
<b>Movement settings:</b> You can change the movement speed across the path, the repositioning and the redirectioning speed in the menu to the right, make sure to hold down [alt] while moving the bars, or you might put down points as you interact with the menu.<br/>
