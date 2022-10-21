# Scannet2bag
A python script for converting the Scannet dataset into a rosbag containing depth point clouds and poses

## Usage
You need to replace these lines with your own directory:

1.  Line 58
```
root_dir = "/media/dyn/DYN1/Research/dataset/iSDF/seqs/scene0004_00/"
```
2.  Line 70
```
bag = rosbag.Bag("/media/dyn/DYN1/Research/dataset/iSDF/rosbag/scene_0004.bag", 'w')
```

At present, the pose in the script uses the **/tf** message from _/camera_ to _/world_, you can modify it to be arbitrary, such as the **PoseStamped** message.
