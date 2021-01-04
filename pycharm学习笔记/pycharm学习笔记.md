pycharm学习笔记 

## pycharm里面无法使用搜狗输入法加入中文注释

第1种方法

在代码开始加入：

```
#!/usr/bin/python
# -*- coding: utf-8 -*-
```

但是有时会失效

第2种方法

1）找到你安装pycharm的安装包路径和bin文件的目录

我的bin文件在写个路径下面

```
/opt/pycharm-2018.1.4/bin
```

2）在当前路径下面ls找到pycharm.sh可执行文件，打开

```
sudo gedit pycharm.sh
```

3）打开后，输入下面这段代码

```
export CLASSPATH

LD_LIBRARY_PATH="$IDE_BIN_HOME:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH
export GTK_IM_MODULE=fcitx
export QT_IM_MODULE=fcitx
export XMODIFIERS=@im=fcitx
```

亲测有效

