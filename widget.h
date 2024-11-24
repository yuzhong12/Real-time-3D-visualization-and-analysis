/**
 * @file widget.h
 * @author Kang Bingxiang， zhou jie
 * @date 2024-10-06
 *
 * @brief Implementation of the Widget class, which provides functionalities for
 *        loading, displaying, and interacting with 3D point clouds and polygon meshes
 *        using the PCL (Point Cloud Library) and Qt framework.
 *
 * @details This file defines the constructor and methods for the Widget class.
 *          It initializes a user interface, sets up default parameters for point cloud
 *          visualization, and implements features such as:
 *          - Loading `.ply` point cloud files.
 *          - Splitting large point clouds into manageable chunks.
 *          - Rendering and interacting with 3D data in a Qt-based GUI.
 *          - Measuring performance for various operations using `QElapsedTimer`.
 *
 *          The class integrates PCL for 3D data handling and VTK for rendering,
 *          making it suitable for applications like real-time 3D visualization and analysis.
 */

#ifndef WIDGET_H
#define WIDGET_H

// Qt Header File
#include <QWidget>
#include <QTime>
#include <QDebug>
#include <QFileDialog>
#include <QColorDialog>
#include <QVTKOpenGLNativeWidget.h>
// PCL Header File
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
// VTK Header File
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkCamera.h>
// C++ Standard Header File ：For std::sqrt
#include <cmath>
#include <string>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget {
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

// 按钮槽函数
private slots:
    void onLoadPlyclicked();     // 点击加载 PLY 文件后调用的槽函数
    void onLoadMeshclicked();    // 点击加载 MESH 文件后调用的槽函数
    void changeBackgroundColor();   // 更改视图的背景颜色
    void changePointCloudColor();   // 更改视图的背景颜色
    void restoreOriginalColor();    // 恢复点云颜色
    void onPointSizeChanged(int value); // 更改点云大小

private:
    Ui::Widget *ui;         // 创建 ui 指针
    int current_chunk;      // 当前点云块
    int num_chunks;         // 点云块数量
    int defaultChunkSize;   // 默认点云块点云数量
    int currentPointSize;   // 当前点云大小
    bool allChunksLoaded;   // 点云加载完整信号
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;  // 创建  RGB  点云
    pcl::PolygonMesh::Ptr mesh;                    // 创建  mesh 对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d; // 创建鼠标点击 RGB 点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_color_cloud;  // 恢复原始颜色点云
    pcl::visualization::PCLVisualizer::Ptr viewer; // 创建视图

    void pickPointCallback(const pcl::visualization::PointPickingEvent& event, void* args); // 回调函数：处理用户点击点云中的点
};

#endif // WIDGET_H
