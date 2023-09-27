# WACV2023 High Resolution Depth Estimation for 360deg Panoramas Through Perspective and Panoramic Dep
# WACV2023 Required:
Visual Studio >= 2019 (recommended)

Getting the project
1. Getting the sources shell
```
git clone https://github.com/Yunwei729/WACV2023-High-Resolution-Depth-Estimation-for-360deg-Panoramas-Through-Perspective-and-Panoramic-Dep.git
```
# To start
2. create folder 
```
 |-- WACV2022
     |-- rgb
         |-- 000d2cac6cfd.jpg
         |-- 000d2cac7cfd.jpg
         |-- ......
     |-- gt
         |-- 000d2cac6cfd.jpg
         |-- 000d2cac7cfd.jpg
         |-- ......
     |-- baseline
	 |-- 000d2cac6cfd.jpg
         |-- 000d2cac7cfd.jpg
         |-- ......
     |-- result
     |-- test_images
```
# And you can run the command
3. run main.cpp with command-line arguments
```
 g++ -o myprogram main.cpp
```
```
 ./myprogram 0 .\\rgb\\ .\\gt\\ .\\baseline\\ .\\result\\
```
# Citation
Please cite our paper for any purpose of usage：
```
@InProceedings{Peng_2023_WACV,
author = {Peng, Chi-Han and Zhang, Jiayao},
title = {High-Resolution Depth Estimation for 360deg Panoramas Through Perspective and Panoramic Depth Images Registration},
booktitle = {Proceedings of the IEEE/CVF Winter Conference on Applications of Computer Vision (WACV)},
month = {January},
year = {2023},
pages = {3116-3125}
}
```