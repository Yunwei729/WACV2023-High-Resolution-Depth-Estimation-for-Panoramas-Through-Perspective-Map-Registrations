# WACV2022
# WACV2022 Required:
Visual Studio >= 2019 (recommended)

Getting the project
1. Getting the sources shell
git clone 

# To start
2.create folder 
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

2.run main.cpp with command-line arguments
```
g++ -o myprogram main.cpp
```
```
./myprogram 0 .\\rgb\\ .\\gt\\ .\\baseline\\ .\\result\\
```
