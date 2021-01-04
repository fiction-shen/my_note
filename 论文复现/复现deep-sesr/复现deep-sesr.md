复现deep-sesr

1.运行keras版本

`Python test_sesr_Keras.py`

报错  Could not create cudnn handle: CUDNN_STATUS_INTERNAL_ERROR

于是在py文件里面加入

`from tensorflow.compat.v1 import ConfigProto`
`from tensorflow.compat.v1 import InteractiveSession`

`config = ConfigProto()`
`config.gpu_options.allow_growth = True`
`session = InteractiveSession(config=config)`

运行成功！

2）TF版本

报错  AttributeError: module 'tensorflow' has no attribute 'GraphDef'等

这些原因是因为在tensorflow2版本用1的API，更改成 

`tf.compat.v1.函数接口`这种形式

然后

`import tensorflow as tf`
`config = tf.compat.v1.ConfigProto(gpu_options=tf.compat.v1.GPUOptions(allow_growth=True))`
`sess = tf.compat.v1.Session(config=config)`

添加这几句，运行成功！