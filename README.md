## 功能描述

用于在 RViz2 中显示 PCD 点云文件，支持通过 YAML 配置文件设置 TF 变换

## 使用方法

    将 rmul_2024.pcd 文件放入 pcd_viewer_ws/src/pcd_viewer/ 目录
文件链接:https://flowus.cn/xlqmu/5ad5d3a4-108c-490f-a5c0-7990ab0a4969

    修改 config/transform.yaml 设置变换参数
    支持x,y,z方向上的平移(单位:m) 和 绕三个轴的旋转(弧度制)

## 构建：
    git clone https://github.com/KokoColaa/pcd_viewer_ws.git
## 
    cd pcd_viewer_ws/
    colcon build --symlink-install
    source install/setup.bash

## 运行：

    ros2 launch pcd_viewer viewer.launch.py
