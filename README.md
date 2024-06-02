# UGVC 2024 ROS1 Workspace
This is official ROS1 workspace of [UGV-DTU](https://sites.google.com/dtu.ac.in/ugvdtu) for [ICTMC UGVC 2024](https://ugvc.conferences.ekb.eg/)

## How to use this repo?

```console
git clone https://github.com/Kartik-Walia/ugvc2024_ws
cd ugvc2024_ws
catkin_make
source devel/setup.bash
```

Remember, you'll need to source devel/setup.bash every time you open a new terminal window. To automate this process and avoid manual sourcing, you can append the following line to your ~/.bashrc file (or equivalent configuration file if you're using a different terminal like zsh):

```
echo "source ./path_to_/ugvc2024_ws/devel/setup.bash" >> ~/.bashrc
```

After adding this line, either restart your terminal or run source ~/.bashrc to apply the changes.
