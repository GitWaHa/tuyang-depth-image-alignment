# tuyang-depth-image-alignment
- 此仓库用于图漾深度相机，目的是将原始深度图像对齐到彩色图像中
- 仅仅适用于FM810-IX-AK4，因为这款相机，官方代码是没有对齐的，而其他型号相机则没有这个问题

## 环境
- 图漾相机官方环境

## 使用
1. 直接使用此功能包，将订阅原始话题，发布对齐后的话题
```bash
# 直接运行launch文件
roslaunch tuyang-depth-image-alignment alignment_server.launch

# 重要参数如下
<param name="topicColorInfo" value="/camera/rgb/camera_info" />
<param name="topicDepthInfo" value="/camera/depth/camera_info" />
<param name="topicDepth" value="/camera/depth/image_raw" />
<param name="topicAlignDepth" value="/camera/depth/image_align" />
```


2. 使用Alignment类，嵌入到应用程序中
```bash
# 详细使用方法参考alignment_server.cpp 中的subCallBack函数
```
