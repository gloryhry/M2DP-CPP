/*
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐│
 *  ││Esc│!1 │@2 │#3 │$4 │%5 │^6 │&7 │*8 │(9 │)0 │_- │+= │|\ │`~ ││
 *  │├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴───┤│
 *  ││ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{[ │}] │ BS  ││
 *  │├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤│
 *  ││ Ctrl │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│" '│ Enter  ││
 *  │├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────┬───┤│
 *  ││ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│Shift │Fn ││
 *  │└─────┬──┴┬──┴──┬┴───┴───┴───┴───┴───┴──┬┴───┴┬──┴┬─────┴───┘│
 *  │      │Fn │ Alt │         Space         │ Alt │Win│   HHKB   │
 *  │      └───┴─────┴───────────────────────┴─────┴───┘          │
 *  └─────────────────────────────────────────────────────────────┘
 * 
 * @Author: Glory Huang
 * @Date: 2021-10-08 09:37:24
 * @LastEditors: Glory Huang
 * @LastEditTime: 2021-10-12 17:43:13
 * @Page: http://gloryhry.github.io/
 * @Github: https://github.com/gloryhry
 * @Description: file content
 */

#include <iostream>
#include <m2dp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <cmath>

using namespace std;

M2DP::M2DP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // this->cloud = *new pcl::PointCloud<pcl::PointXYZ>();
    pcl::copyPointCloud(*cloud, *this->cloud);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    pca.project(*cloud, *this->cloud_filtered);
    for (int i = 0; i < this->cloud_filtered->points.size();i++)
    {
        this->cloud_filtered->points[i].z = -this->cloud_filtered->points[i].z;
    }
    cloud_pca.resize(this->cloud_filtered->points.size(), 3);
    for (int i = 0; i < this->cloud_filtered->points.size(); i++)
    {
        cloud_pca(i, 0) = this->cloud_filtered->points[i].x;
        cloud_pca(i, 1) = this->cloud_filtered->points[i].y;
        cloud_pca(i, 2) = this->cloud_filtered->points[i].z;
    }
    azimuthList = *new vector<double>(numP);
    for (int i = 0; i < numP; i++)
    {
        azimuthList[i] = -M_PI_2 + i * M_PI / (numP - 1);
    }
    elevationList = *new vector<double>(numQ);
    for (int i = 0; i < numQ; i++)
    {
        elevationList[i] = i * M_PI_2 / (numQ - 1);
    }
    // get the farthest point distance
    for (auto pt : this->cloud_filtered->points)
    {
        double temp_rho = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (temp_rho > maxRho)
        {
            maxRho = temp_rho;
        }
    }
    // main function, get the signature matrix A
    A = GetSignatureMatrix();
    // Eigen::JacobiSVD<Eigen::Matrix<double, 64, 128>> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd u = svd.matrixU();
    Eigen::MatrixXd v = svd.matrixV();
    Eigen::Matrix<double, 1, 64> u_temp;
    Eigen::Matrix<double, 1, 128> v_temp;
    u_temp = u.col(0);
    v_temp = v.col(0);
    m2dp_result << u_temp, v_temp;
}

Eigen::Matrix<double, 64, 128> M2DP::GetSignatureMatrix()
{
    vector<double> thetaList(numT + 1);
    for (int i = 0; i <= numT; i++)
    {
        thetaList[i] = -M_PI + i * 2 * M_PI / (numT);
    }
    vector<double> rhoList(numR + 1);
    for (int i = 0; i <= numR; i++)
    {
        rhoList[i] = i * sqrt(maxRho) / numR;
        rhoList[i] = rhoList[i] * rhoList[i];
    }
    // make sure all points in bins
    rhoList[rhoList.size() - 1] = rhoList[rhoList.size() - 1] + 0.001; 
    
    Eigen::Matrix<double, 64, 128> result_A;
    int index_A = 0;
    // loop on azimuth
    for (int i = 0; i < azimuthList.size();i++)
    {
        auto azm = azimuthList[i];
        // loop on elevation
        for (int j = 0; j < elevationList.size();j++)
        {
            auto elv = elevationList[j];
            // normal vector vecN of the selected 2D plane
            Eigen::Matrix<double, 1, 3> vecN;
            sph2cart(azm, elv, 1, vecN);
            // distance of vector [1,0,0] to the surface with normal vector vecN
            Eigen::Matrix<double, 1, 3> op(1, 0, 0);
            Eigen::MatrixXd h = op * vecN.transpose();
            // a new vector, c = h*vecN, so that vector [1,0,0]-c is the projection of x-axis onto the plane with normal vector vecN
            Eigen::Matrix<double, 1, 3> c = h(0) * vecN;
            // x-axis - c, the projection
            Eigen::Matrix<double, 1, 3> px = op - c;
            // given the normal vector vecN and the projected x-axis px, the y- axis is cross(vecN,px)
            Eigen::Matrix<double, 1, 3> py = vecN.cross(px);
            // projection of data onto space span{px,py}
            Eigen::MatrixXd pcx = cloud_pca * px.transpose();
            Eigen::MatrixXd pcy = cloud_pca * py.transpose();
            // pdata = np.array([pcx,pcy])
            // represent data in polar coordinates
            // vector<double> rho;
            // vector<double> theta;
            vector<Eigen::Vector2d> points; // x: rho  y: theta
            for (int i = 0; i < pcx.rows(); i++)
            {
                Eigen::Vector2d temp;
                cart2pol(pcx(i), pcy(i), temp);
                points.push_back(temp);
            }
            // main function, count points in bins
            Eigen::MatrixXd hist; //16*8    thetaList 17   rhoList 9
            histogram2d(points, thetaList, rhoList, hist);
            hist = hist / cloud_filtered->points.size();
            int hist_size = hist.cols() * hist.rows();
            for (int i = 0; i < hist_size; i++)
            {
                result_A(index_A, i) = hist(i);
            }
            index_A++;
        }
    }
    return result_A;
}

void M2DP::cart2sph(double &azm, double &elv, double &r, Eigen::Vector3d vecN)
{
    azm = atan2(vecN.y(), vecN.x());
    elv = atan2(vecN.z(), sqrt(vecN.x() * vecN.x() + vecN.y() * vecN.y()));
    r = sqrt(vecN.x() * vecN.x() + vecN.y() * vecN.y() + vecN.z() + vecN.z());
}

void M2DP::sph2cart(double azm, double elv, double r, Eigen::Matrix<double, 1, 3> &vecN)
{
    double x, y, z;
    x = r * cos(elv) * cos(azm);
    y = r * cos(elv) * sin(azm);
    z = r * sin(elv);
    vecN << x, y, z;
}

void M2DP::cart2pol(double x, double y, Eigen::Vector2d &vecN)
{
    vecN.x() = sqrt(x * x + y * y); // rho
    vecN.y() = atan2(y, x);         // phi
}

void M2DP::pol2cart(double rho, double phi, Eigen::Vector2d &vecN)
{
    double x, y;
    x = rho * cos(phi);
    y = rho * sin(phi);
    vecN << x, y;
}

void M2DP::histogram2d(vector<Eigen::Vector2d> points, vector<double> thetaList, vector<double> rhoList, Eigen::MatrixXd &hist)
{
    int row, col;
    row = thetaList.size() - 1;
    col = rhoList.size() - 1;
    hist = Eigen::MatrixXd::Zero(row, col);
    // Points x: rho  y: theta
    for (auto pt : points)
    {
        int row_index = -1, col_index = -1;
        for (int i = 0; i <= row; i++)
        {
            if (pt.y() < thetaList[i])
            {
                row_index = i - 1;
                break;
            }
        }
        for (int j = 0; j <= col; j++)
        {
            if (pt.x() < rhoList[j])
            {
                col_index = j - 1;
                break;
            }
        }
        if (row_index >= 0 && row_index < row && col_index >= 0 && col_index < col)
        {
            hist(row_index, col_index)++;
        }
    }
}