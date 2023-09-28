# WACV2023-High Resolution Depth Estimation for Panoramas Through Perspective Map Registrations
High-Resolution Depth Estimation for 360deg Panoramas Through Perspective and Panoramic Depth Images Registration <br>
Chi-Han Peng and Jiayao Zhang <br>
IEEE/CVF Winter Conference on Applications of Computer Vision (WACV), 2023 <br>
[Paper](https://pengchihan.co/papers/Peng_High-Resolution_Depth_Estimation_for_360deg_Panoramas_Through_Perspective_and_Panoramic_WACV_2023_paper.pdf) ,
[Supplemental](https://pengchihan.co/papers/Peng_High-Resolution_Depth_Estimation_WACV_2023_supplemental.pdf) ,
[Poster](https://pengchihan.co/papers/1428-wacv-post.pdf) ,
[Video](https://pengchihan.co/papers/1428-wacv.mp4)

# Methodology
![127](https://github.com/Yunwei729/WACV2023-High-Resolution-Depth-Estimation-for-Panoramas-Through-Perspective-Map-Registrations/assets/77334402/bae79da0-8c58-4abc-a93f-3c6cc6723442)


# Datasets
In this project, we introduce the term 'baseline,' which specifically refers to depth maps generated using existing computer vision methods such as [Unifuse](https://github.com/alibaba/UniFuse-Unidirectional-Fusion) or [HoHoNet](https://github.com/sunset1995/HoHoNet). It is used for comparing and evaluating the performance of other depth map generation algorithms.

# WACV2023 Requirements:
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
