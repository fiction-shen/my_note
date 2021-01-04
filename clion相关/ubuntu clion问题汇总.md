# 问题 ubuntu16.04中clion 中不能输入中文

Fcitx，搜狗输入法，不能输入中文

## 解决

[在clion的安装目录下找到bin文件夹下面的clion.sh](http://xn--clionbinclion-t40uda378adxs8xvr5id4lh6tt2pzoq6f0hma85yd58qfszd.sh/)，然后找到

```
# ---------------------------------------------------------------------
# Run the IDE.
# ---------------------------------------------------------------------
```

在上面添加

```
XMODIFIERS="@im=fcitx"
export XMODIFIERS
```

重启即可。