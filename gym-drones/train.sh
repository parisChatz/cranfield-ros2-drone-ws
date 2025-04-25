# source ~/cranfield-ros2-drone-ws/venv/bin/activate
source ~/cranfield-ros2-drone-ws/install/setup.bash 

# exports are included in the launch file.
# export GZ_SIM_RESOURCE_PATH=$HOME/cranfield-ros2-drone-ws/src/my_drone_sim/models:$HOME/cranfield-ros2-drone-ws/src/my_drone_sim/worlds:$GZ

python3 train.py

# We want something like this:
# python3 train.py --env-name GazeboDrone-v0 --num-episodes 1000 --num-steps 1000 --log-dir ./logs --save-dir ./models --load-dir ./models --save-freq 10 --load-freq 10 --eval-freq 10 --eval-episodes 5 --eval-log-dir ./eval_logs