# 将点云文件保存文pcd文件

```c++
    void saveAsPCD()    //将文档保存为PCD
    {
        sprintf(savepcdpath, "pcd/%03d.pcd", ++savepcdcount);
        //写文件声明
        FILE *writePCDStream = fopen(savepcdpath, "wb");
        fprintf(writePCDStream, "VERSION 0.7\n");                 //版本说明
        fprintf(writePCDStream, "FIELDS x y z\n");                //维度说明
        fprintf(writePCDStream, "SIZE 4 4 4\n");                  //占用字节说明
        fprintf(writePCDStream, "TYPE F F F\n");                  //具体数据类型定义
        fprintf(writePCDStream, "WIDTH %d\n", circlelen * LINE);  //点数量
        fprintf(writePCDStream, "HEIGHT 1\n");                    //无序点云默认为1
        fprintf(writePCDStream, "POINTS %d\n", circlelen * LINE); //点数量
        fprintf(writePCDStream, "DATA ascii\n");                  //文档使用字符类型shuom
        //写点云数据
        for (size_t l = 0; l < LINE; l++)
        {
            for (int32_t c = 0; c < circlelen; c++)
            {
                fprintf(writePCDStream, "%f %f %f\n", mptclout[l][c].x, mptclout[l][c].y, mptclout[l][c].z);
            }
        }
        fclose(writePCDStream);
    }
};
```



# 点云数据pcd文件简介

## 1. 什么是点云数据

 点云数据是指在**一个三维坐标系**中的一组向量的集合。这些向量通常以**X，Ｙ，Z**三维坐标的形式表示，一般主要代表一个物体的外表面几何形状，除此之外点云数据还可以附带RGB信息，即每个坐标点的颜色信息，或者是其他的信息。

## 2. 点云数据格式——PCD文件

在这里推荐大家都将都将点云数据保存为 *.pcd文件，因为有pcl这个开源库专门处理pcd格式的文件，它实现了点云获取、滤波、分割、配准、检索、特征提取、识别、追踪、曲面重建、可视化等。

下面是通过pycharm编辑器解析的.pcd文件：

![pcd文件解析](https://img-blog.csdnimg.cn/20190812095020886.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQzMDQ5NDMy,size_16,color_FFFFFF,t_70#pic_center)

### 2.1 下面介绍该文件每行具体所代表的含义：

(1).VERSION —— 指定PCD文件版本

​	  例如 0.7

(2).FIELDS —— 指定一个点可以有的每一个维度和字段的名字。
    例如：FIELDS x y z r g b表示该点的位置信息（x，y，z），颜色信息（r，g，b）。

​		FIELDS x y z                  # XYZ data

​		FIELDS x y z rgb             # XYZ + colors

​		FIELDS x y z normal_xnormal_y normal_z     # XYZ + surface normals

​		FIELDS j1 j2 j3                # moment invariants

(3).SIZE —— 用字节数指定每一个维度的大小

(4).TYPE —— 用一个字符指定每一个维度的类型
    I : 可表示int8,int16,int32。
    U : 可表示uint8,unit16,uint32。
    F : 表示float（上图所用的为浮点类型）。

(5).COUNT —— 指定每一个维度包含的元素数目
    例如xyz数据通常只有一个元素。

(6).WIDTH —— 用点的数量表示点云数据集的宽度。根据是有序点云还是无序点云，WIDTH有两层解释：
    1.可指定点云总个数(与POINTS相同)，用于无组织的数据。
    2.可指定有组织点云数据的宽度(连续点的总数)。

(7).HEIGTH —— 用点的数目表示点云数据集的高度。类似于WIDTH ，HEIGHT也有两层解释：
    1.可指定有组织的点云数据的高度(总行数)。
    2.对未组织的数据,它被设置为1。

​	  有序点云例子：

​        WIDTH 640    # 像图像一样的有序结构，有640行和480列，

​        HEIGHT 480   # 这样该数据集中共有640*480=307200个点

​      无序点云例子：

​        WIDTH 307200

​        HEIGHT 1    # 有307200个点的无序点云数据集

(8).VIEWPOINT—— 指定数据集中点云的获取视点。

​        VIEWPOINT有可能在不同坐标系之间转换的时候应用，在辅助获取其他特征时也比较有用，例如曲面法线，在判断方向一致性时，需要知道视点的方位，采集数据时的视点(由平移tx,ty,tz和四元数qw,qx,qy,qz组成)。

​        视点信息被指定为平移（txtytz）+四元数（qwqxqyqz）。默认值是：

​         VIEWPOINT 0 0 0 1 0 0 0

(9).POINTS —— 指定点云总个数

​       从0.7版本开始，该字段就有点多余了，因此有可能在将来的版本中将它移除。

(10).DATA —— 指定存储点云数据的数据类型
    从0.7版本开始，支持两种数据类型：ascii和二进制。

注意：文件头最后一行（DATA）的下一个字节就被看成是点云的数据部分了，它会被解释为点云数据。

警告：PCD文件的文件头部分必须以上面的顺序精确指定，也就是如下顺序：

VERSION、FIELDS、SIZE、TYPE、COUNT、WIDTH、HEIGHT、VIEWPOINT、POINTS、DATA

之间用换行隔开。