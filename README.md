# TODO：增加检测算法



# 目标跟踪

算法：KCF

效果：

![xyzEstimation](README.assets/xyzEstimation_KCF.png) [estimateQuadcopterLoad-2020-02-19_17.10.04.mkv](videos/estimateQuadcopterLoad-2020-02-19_17.10.04.mkv) 

# 防摆控制算法

PD控制

https://www.zhihu.com/answer/330795289

想法将速度影响的比例增大，使得达到文章中与人相似的效果

TODO: 尝试结合位置差和速度拟合成一个输出



## 控制俯仰角和翻转角的PD控制

无控制

## ![xvControl](README.assets/without_xvControl.png)

有控制

![xvControl](README.assets/xvControl.png)

## 控制升力F和力矩M的飞控控制法

无控制

![payloadWithoutControl_PrecisePos](README.assets/payloadWithoutControl_PrecisePos.png)

有控制

![payloadControl_PrecisePos](README.assets/payloadControl_PrecisePos.png)

## iLQR控制升力F和力矩M的控制