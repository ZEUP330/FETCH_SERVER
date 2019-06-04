# training fetch reach the certain goal in gazebo
## we had used the DQN for fetch reach trail
but the [performance](https://github.com/ZEUP330/FETCH_SERVER/tree/master/DQN_SERVER) don't well
## Using DDPG for training fetch reach goal
you can find fetch and environment class in [here](https://github.com/ZEUP330/FETCH_SERVER/tree/master/DDPG_SERVER/GazeboEnv.py)

DDPG algorithm in [here](https://github.com/ZEUP330/FETCH_SERVER/tree/master/DDPG_SERVER/SimpleDDPG.py)

main function entrance in [here](https://github.com/ZEUP330/FETCH_SERVER/tree/master/DDPG_SERVER/ddpgmain.py)

# RGB+D to xyz
we have trained a [network](https://github.com/ZEUP330/FETCH_SERVER/tree/master/RGBD_XYZ) which can get xyz in simulate(gazebo) for img with deep.

there have a [fetch_gazebo](https://github.com/ZEUP330/FETCH_SERVER/tree/master/fetch_gazebo) world (including obstacle model, fetch and the world)

if you are ubuntu you can try 

```shell
sudo apt-get install ros-indigo-fetch_*
```

then you will install fetch package in /opt/ros/indigo/share/...(i copy the package from here);


## you can get xyz from img by:
```shell
python demo.py

```

# Using Multi-GPU:

Except put your data and model in GPU, you need to using follow coda:
```python
device_ids = [0, 1]
net = Net().cuda(device_ids[0])
net = nn.DataParallel(net, device_ids=device_ids)
optimizer = torch.optim.Adam(eval_net.parameters(), lr=LR) 
optimizer = nn.DataParallel(optimizer, device_ids=device_ids)
optimizer.module.step()

```
