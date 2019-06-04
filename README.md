# training fetch reach the certain goal in gazebo
## we had used the DQN for fetch reach trail
but the [performance](https://github.com/ZEUP330/FETCH_SERVER/tree/master/DQN_SERVER) don't well
## Using DDPG for training fetch reach goal
[here](https://github.com/ZEUP330/FETCH_SERVER/tree/master/DDPG_SERVER)
# RGB+D to xyz
we have trained a [network](https://github.com/ZEUP330/FETCH_SERVER/tree/master/RGBD_XYZ) which can get xyz in simulate(gazebo) for img with deep.

##you can get xyz from img by:
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
