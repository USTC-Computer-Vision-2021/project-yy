# 基于SIFT算法的增强现实

喻言 PB18061165

## 问题描述

增强现实（AR）是一种将三维的现实与二维的图像融合的技术。他可以帮助我们正确判断图片场景中的空间结构，并将3D模型合理地投影在二维平面中，形成二维与三维融合的图片。增强现实技术可以极大地提高图像的趣味性、生动性，实现将虚拟世界投影到现实世界的效果。

本项目为基于SIFT算法的增强现实，利用SIFT将源图片在目标图片中定位，并计算源图片的单应性矩阵和相机矩阵。之后，在目标图片上投影预设的模型。本项目实现了obj文件的边框读取和输出，因而可以投影obj类型的3d模型。

## 原理分析

### SIFT算法

SIFT算法是本项目实现的基础，其基本原理为尺度不变特征变换。根据图像尺度不变的特性，在图片中提取特征点，并进一步计算特征点的方向。SIFT算法主要有以下步骤：

1. **提取关键点：**关键点是一些十分突出的不会因光照、尺度、旋转等因素而消失的点，比如角点、边缘点、暗区域的亮点以及亮区域的暗点。此步骤是搜索所有尺度空间上的图像位置。通过高斯微分函数来识别潜在的具有尺度和旋转不变的兴趣点。
2. **定位关键点并确定特征方向：**在每个候选的位置上，通过一个拟合精细的模型来确定位置和尺度。关键点的选择依据于它们的稳定程度。然后基于图像局部的梯度方向，分配给每个关键点位置一个或多个方向。所有后面的对图像数据的操作都相对于关键点的方向、尺度和位置进行变换，从而提供对于这些变换的不变性。

3. **特征匹配：**通过各关键点的特征向量，进行两两比较找出相互匹配的若干对特征点，建立景物间的对应关系。

#### 1 提取关键点

##### 1.1 高斯金字塔

提取关键点首先需要构建尺度空间，使原图像保存最多的细节特征。由于高斯核函数是唯一的尺度不变核函数，此处使用构建高斯金字塔的方法，来获取不同尺度下图像的细节特征。通过使用不同方差的高斯核函数，对原图像进行滤波，可以在不同细节尺度下得到图像的细节特征，高斯卷积核函数为：
$$
\begin{align}
& G(x_{i},y_i,\sigma) = \frac{1}{2\pi\sigma^{2}}e^{-\frac{(x-x_{i})^2+(y-y_i)^2}{2\sigma}}\\
& L(x,y,\sigma) = G(x,y,\sigma) * I(x,y)
\end{align}
$$

使用不同$\sigma$的高斯函数对图像进行滤波，可以得到高斯金字塔。对于共有$O$组，每组$S$层的高斯金字塔，其第$i$组第$r$层的方差$\sigma(i,r)$为：
$$
\sigma(i,r)=\sigma_02^{i+r/S}, o\in[0,\cdots,O-1],i\in[0,\cdots,S-1],
$$
在计算组内某一层图像的尺度时，直接使用如下公式进行计算：
$$
\sigma(r) = \sigma_{0}2^{r/s}
$$

##### 1.2 DoG空间极值检测

高斯差分函数（Difference of Gaussians）通过对高斯金字塔各层进行差分运算，可以描绘出目标的轮廓。DoG函数为：
$$
D(x,y,\sigma)=[G(x,y,k\sigma)-G(x,y,\sigma)]*I(x,y)
$$
使用DoG算子，对高斯金字塔的每一组内进行差分运算，得到DoG金字塔。图像的特征点是由DoG空间的极值点组成的。因而寻找特征点，需要对DoG中每个点与其相邻和上下共计26个点进行比较，确保在尺度空间和二维图像空间都能检测到极值点。

##### 1.3 去除边缘效应

由于DoG算子会产生较强的边缘响应，去除低对比度的关键点和不稳定的边缘响应点，可以增强匹配稳定性、提高抗噪声能力。候选特征点的DoG函数值的主曲率与$2\times2Hessian$矩阵$H$的特征值成正比。$Hessian$矩阵为：
$$
H = \begin{bmatrix}  
  D_{xx} & D_{xy} \\  
  D_{xy} & D_{yy}
\end{bmatrix}
$$


其中，$D_{xx},D_{xy},D_{yy}$是候选点邻域对应位置的差分求得的。矩阵$H$的特征值$\alpha$和$β$代表$x$和$y$方向的梯度，计算如下：
$$
\begin{align}
& Tr(H)=D_{xx}+D_{yy}=\alpha+\beta\\
& Det(H)=D_{xx}D_{yy}-D_{xy}^{2} = \alpha*\beta
\end{align}
$$
令$\alpha = r \beta$，当两特征值相差较大时，即在某一个方向的梯度值越大，而在另一个方向的梯度值越小，而边缘恰恰就是这种情况，则将该特征点剔除。由于：
$$
\frac{Tr(H)^2}{Det(H)}=\frac{(\alpha+\beta)^2}{\alpha\beta} = \frac{(r+1)^{2}}{r}
$$
令$r_0=10$，只需进行比较：
$$
\frac{Tr(H)^2}{Det(H)}<\frac{(r_0+1)^{2}}{r_0}
$$
即可判定被剔除的特征点。

#### 2 确定关键点并确定特征方向

通过尺度不变性求极值点，需要利用图像的局部特征为给每一个关键点分配一个基准方向，使描述子对图像旋转具有不变性。对于在DoG金字塔中检测出的关键点，采集其所在高斯金字塔图像$3\sigma$邻域窗口内像素的梯度和方向分布特征。梯度的模值和方向如下：
$$
\begin{align}
& m(x,y)=\sqrt{(L(x+1,y)-L(x-1,y))^2+(L(x,y+1)-L(x,y-1))^2}\\
& \theta(x,y) = \tan^{-1}(L(x,y+1)-L(x,y-1))/(L(x+1,y)-L(x-1,y))
\end{align}
$$


本算法采用梯度直方图统计法，统计以关键点为原点，一定区域内的图像像素点确定关键点方向。在完成关键点的梯度计算后，使用直方图统计邻域内像素的梯度和方向。梯度直方图将$0\sim360$度的方向范围分为$36$个柱，其中每柱$10$度。直方图的峰值方向代表了关键点的主方向，方向直方图的峰值则代表了该特征点处邻域梯度的方向，以直方图中最大值作为该关键点的主方向。为了增强匹配的鲁棒性，只保留峰值大于主方向峰值$80％$的方向作为该关键点的辅方向。

#### 3 特征匹配

##### 3.1 关键点描述

通过以上步骤，对于每一个关键点，拥有三个信息：位置(pt)、尺度(size)以及方向(orientation)。接下来就是为每个关键点建立一个描述符，使其不随各种变化而改变，比如光照变化、视角变化等等。并且描述符应该有较高的独特性，以便于提高特征点正确匹配的概率。

将关键点附近的区域划分为$d\times d$个子区域，每个子区域作为一个种子点，每个种子点有8个方向。考虑到实际计算时，需要采用三线性插值，所需图像窗口边长为$3\times 3\times σ_{oct}(d+1)$。在考虑到旋转因素(方便下一步将坐标轴旋转到关键点的方向)，实际计算所需的图像区域半径为：
$$
radius=\frac{3\sigma_{oct}\times \sqrt{2}\times (d+1)}{2}
$$
计算结果四舍五入取整。之后将坐标轴旋转为关键点的方向，以确保旋转不变性。旋转后邻域内采样点的新坐标为：
$$
\begin{pmatrix}  
  x'\\  
  y' 
\end{pmatrix} = 
\begin{pmatrix}  
	cos\theta & −sin\theta \\
	sin\theta & cos\theta
\end{pmatrix} 
\quad (x,y\in[-radius,radius])
$$
之后将邻域内的采样点分配到对应的子区域内，将子区域内的梯度值分配到8个方向上，计算其权值。

旋转后的采样点坐标在半径为$radius$的圆内被分配到$d\times d$的子区域，计算影响子区域的采样点的梯度和方向，分配到8个方向上。旋转后的采样点$(x',y')$落在子区域的下标为：
$$
\begin{pmatrix}  
  x''\\  
  y'' 
\end{pmatrix} =  \frac{1}{3\sigma_{oct}}
\begin{pmatrix}  
  x'\\  
  y' 
\end{pmatrix} = +\frac{d}{2}
$$
像素的梯度大小按$ \sigma=0.5d$的高斯加权计算，即：
$$
w=m(a+x,b+y)*e^{-\frac{(x')^2+(y')^2}{2\times (0.5d)^2}}
$$

其中$a,b$为关键点在高斯金字塔图像中的位置坐标。实际计算$r$时，使用的区域为$ d+1 $，在实际计算时$+0.5$，保证四舍五入，加而不减保证参加统计的数据尽可能多，同时为了平衡，在计算$(x'',y'')$时$-0.5$。

将所得采样点在子区域中的下标(图中蓝色窗口内红色点)线性插值，计算其对每个种子点的贡献。则最终累加在每个方向上的梯度大小为：
$$
weight=|grad(I_{\sigma}(x,y))|\times e^{-\frac{x_k^2+y_k^2}{2\sigma_w}} \times (1-d_r)\times (1-d_c) \times(1-d_o)
$$
其中，$x_k$为该点与关键点的列距离，$y_k$为该点与关键点的行距离，$\sigma_w$等于描述子窗口宽度$\frac{3\sigma d}{2} $ 。

如上统计的$4∗4∗8=128$个梯度信息即为该关键点的特征向量。特征向量形成后，为了去除光照变化的影响，需要对它们进行归一化处理，对于图像灰度值整体漂移，图像各点的梯度是邻域像素相减得到，所以也能去除。得到的描述子向量为H = ( h 1 , h 2 , . . . , h 128 ) $H=(h_1,h_2,\cdots,h_{128})$，归一化后的特征向量为$L=(L_1,L_2,\cdots,L_{128})$，则：
$$
L_j=\frac{h_j}{\sum\limits_{i=1}^{128}h_i},\quad j=1,2,3\cdots
$$
描述子向量门限。非线性光照，相机饱和度变化对造成某些方向的梯度值过大，而对方向的影响微弱。因此设置门限值(向量归一化后，一般取$0.2$)截断较大的梯度值。然后，再进行一次归一化处理，提高特征的鉴别性。

##### 3.2 关键点匹配

分别对源图像和目标图像建立关键点描述子集合。目标的识别是通过两点集内关键点描述子的比对来完成。采用欧式距离对具有128维的关键点描述子进行相似性度量。

匹配可采取穷举法完成，但所花费的时间太多。所以一般采用$kd$树的数据结构来完成搜索。搜索的内容是以目标图像的关键点为基准，搜索与目标图像的特征点最邻近的原图像特征点和次邻近的原图像特征点。

本项目关键点匹配采用`PCV.localdescriptors.sift`$中的$`match`函数实现。

## 代码实现

本项目启动文件为**AR.py**，其主函数为：

```python
if __name__ == '__main__':
    for arg in sys.argv[1:]:
        arg = arg.split('=')
        if arg[0] == '--file_name0':
            file_name0 = arg[1]
        elif arg[0] == '--file_name1':
            file_name1 = arg[1]
        elif arg[0] == '--obj_file_name':
            obj_file_name = arg[1]

    ar = AR(file_name0, file_name1, obj_file_name, img_file_path, obj_file_path)
    ar.Frame_Model()
    ar.Solid_3D_Model()
```

$2\sim 9$行用于读取命令行参数，可以在命令行中定义源图片文件名和目标图片文件名，如果有obj文件也可添加。

`AR`类为代码的核心类，在该类中将完成对源图片和目标图片的匹配处理。

`Frame_Model()`函数将显示源图片和目标图片中识别的特征点，使用边框确定其范围，并在目标图片上放置obj模型框架。若未添加obj模型，则会放置一立方体。

`Solid_3D_Model()`函数将在目标图片上放置一个红色茶壶。

由于代码内容较多，不便全部贴在README.md中，因而其余代码均在文件中添加注释进行解释，请前往代码文件中查看。

## 效果展示

2D square projection of bottom square

![1.test-res](Readme_use/2D_square_projection_of_bottom_square.png)

2D square projection transformed with H

![1.test-res](Readme_use/2D_square_projection_transformed_with_H.png)

3D model projected in second image

![1.test-res](Readme_use/3D_model_projected_in_second_image.png)

AR Teapot

![1.test-res](Readme_use/AR_teapot.png)



## 工程结构

```css
.
├── README.md
├── AR.py
├── ARutils.py
├── obj_3d.py
├── SIFT
│   └── pysift.py
├── img_resource
│   └── img_file0
│   └── img_file1
├── obj_resource
│   └── obj_file
├── data
│   └── img_data
└── Readme_use
```

## 运行说明

由于没有使用**opencv**自带的`sift`函数，因而对**opencv**的版本没有过多要求，直接用pip安装即可。

```python
pip install opencv
pip install PCV
pip install pygame
pip install PyOpenGL PyOpenGL_accelerate
python AR.py --file_name0=5G.png --file_name1=5G_com.png --obj_file_name=mk.obj
```

将源图片文件和目标图片文件放入***img_resource***文件夹中，如果需要使用obj模型，则将obj文件放入***obj_resource***文件夹中。

打开**AR.py**时，参数`file_name0`为源图片文件名，`file_name1`为目标图片文件名，`obj_file_name`为obj文件名。

**注意：目标图片的长或宽中至少有一项为$2$的整数次幂，否则无法正确调用`Solid_3D_Model()`函数显示实心3D模型。**

## 参考文献

 [SIFT 原理](https://blog.csdn.net/qq_37374643/article/details/88606351)

[SIFT](https://blog.csdn.net/sakurakawa/article/details/120833167)

[Teapot demo](https://blog.csdn.net/qq_42617827/article/details/89052267)

