# UGVC 2024 ROS1 Workspace
This is official ROS1 workspace of [UGV-DTU](https://sites.google.com/dtu.ac.in/ugvdtu) for [ICTMC UGVC 2024](https://ugvc.conferences.ekb.eg/)

## How to use this repo?

```console
git clone https://github.com/Kartik-Walia/ugvc2024_ws
cd ugvc2024_ws
catkin_make
source devel/setup.bash
```

You need to source devel/setup.bash everytime you open terminal. To avoid this run following line. Make sure to change the path to your folder. This command add `source devel/setup.bash` to your ~/.bashrc.(If you are using different terminal (eg. `zsh`) make sure to replace .bashrc with your respective configuration file (eg. `.zshrc` in case of zsh)).

```
echo "source ./path_to_/ugvc2024_ws/devel/setup.bash" >> ~/.bashrc
```

Now either restart your terminal or run `source ~/.bashrc`
