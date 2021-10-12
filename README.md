# M2DP-CPP

[![license](https://img.shields.io/github/license/gloryhry/M2DP-CPP)](https://github.com/gloryhry/radar2pointcloud/blob/master/LICENSE)

C++ implementation of the M2DP algorithm. Original repository can be found [here](https://github.com/LiHeUA/M2DP).

## Introduction:
 Multiview 2D projection (M2DP) is a global descriptor of input point cloud.
 
 ```
  Input:
       data               pcl::PointCloud<pcl::PointXYZ>::Ptr      Point cloud. 
 Output:
       desM2DP     Eigen::Matrix<double, 192, 1>         M2DP descriptor of the input cloud data
       A                      Eigen::Matrix<double, 64, 128>      Signature matrix
 ```
 Details of M2DP can be found in the following paper:

 Li He, Xiaolong Wang and Hong Zhang, M2DP: A Novel 3D Point Cloud 
 Descriptor and Its Application in Loop Closure Detection, IROS 2016.

 Li He, Dept. of Computing Science, University of Alberta
 lhe2@ualberta.ca
 
## Requirement
- Eigen3
- PCL


## usage 

**Follow the below format to extract features from pointcloud:**

```C++
#include <m2dp.h>

...
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
M2DP m2dp(cloud);
Eigen::MatrixXd desM2dp;
desM2dp = m2dp.get_m2dp_result();
Eigen::MatrixXd A;
A = m2dp.get_m2dp_A();

```

## Reference

		@inproceedings{he2016m2dp,
	  title={M2DP: A novel 3D point cloud descriptor and its application in loop closure detection},
	  author={He, Li and Wang, Xiaolong and Zhang, Hong},
	  booktitle={2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	  pages={231--237},
	  year={2016},
	  organization={IEEE}
	}


## Copyright
See [LICENSE](LICENSE) for details.
