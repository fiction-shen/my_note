# 复现PointNet --Tensorflow版本

代码地址：https://github.com/charlesq34/pointnet

参考blog：https://blog.csdn.net/qq_40234695/article/details/86223577

## 1.下载数据集

由于墙的关系，导致数据集通过代码无法下载，因此手动下载[modelnet40_ply_hdf5_2048](https://shapenet.cs.stanford.edu/media/modelnet40_ply_hdf5_2048.zip)

放到/data文件夹（需要解压）（具体存放位置可通过provide.py代码看到）

## 2.执行train.py

`python3 train.py`

## 3.可能遇到的问题：

1）AttributeError: module 'tensorflow' has no attribute 'placeholder'等

![img](复现pointnet.assets\DC619636DE6F8FD5B83C36D6827939B8.jpg)

原因：在tf2下使用了tf1的API

`import tensorflow as tf`

`tf.__version__`

解决办法：把下面的代码更换

`import tensorflow as tf`  

->>>>>

`import tensorflow.compat.v1 as tf`

`tf.disable_v2_behavior()`

解决办法：把tensorflow降级到1版本(不可行，和cuda版本不匹配)

`sudo pip3 uninstall tensorflow`

`sudo pip3 install tensorflow-gpu==1.2`

2）提示 tensorflow.compat.v1 has no attribute contrib 

解决办法：

把这一句替换为 `tf.truncated_normal_initializer(stddev=0.1)`

## 4.查看训练过程

`tensorboard --logdir log` 通过网址查看训练过程

## 附：代码解析

1 `with tf.Graph().as_default():`

设置tensor所属的graph，具体原理尚不清楚

2 `with tf.device('/gpu:'+str(GPU_INDEX)):`

设置tensor运行硬件环境为GPU

3 `pointclouds_pl, labels_pl = MODEL.placeholder_inputs(BATCH_SIZE, NUM_POINT)`
调用了设置model内的placeholder_inputs函数，默认是pointnet_cls.py，内部函数实现是

`def placeholder_inputs(batch_size, num_point):`
    `pointclouds_pl = tf.placeholder(tf.float32, shape=(batch_size, num_point, 3))`
    `labels_pl = tf.placeholder(tf.int32, shape=(batch_size))`
    `return pointclouds_pl, labels_pl`
调用tf.placeholder构建出输入点的tensor以及标签的tensor，构建一个三维的32x1024x3的tensor，构建一个一维的32的tensor
4 ` is_training_pl = tf.placeholder(tf.bool, shape=())`
构建一个bool型的shape未知的tensor

5 `print(is_trining_pl)`

6 
`batch = tf.Variable(0)`
`bn_decay = get_bn_decay(batch)`
`tf.summary.scalar('bn_decay', bn_decy)`

声明一个tensor常量，值为0
