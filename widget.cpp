/**
 * @file widget.cpp
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



#include "widget.h"
#include "ui_widget.h"

// 构造函数
Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , cloud(new pcl::PointCloud<pcl::PointXYZRGB>())
    , mesh(new pcl::PolygonMesh)
    , clicked_points_3d(new pcl::PointCloud<pcl::PointXYZRGB>())
    , original_color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>())
{
    ui->setupUi(this); // 初始化用户界面

    defaultChunkSize = 1000000; // 设置默认的点云块大小
    allChunksLoaded = false;  // 初始状态，没有加载所有点云块

    // 设置视图窗口属性，自定义渲染
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer->setBackgroundColor(213.0 / 255.0, 220.0 / 255.0, 230.0 / 255.0);
    viewer->addPointCloud(cloud, "cloud");
    viewer->resetCamera();               // 设置视角对准点云模型中心，以便点云全局显示
    viewer->addText("Model Viewer", 10, 10, 20, 1, 0, 0,"3D Model");  // 添加2d文字标签
    viewer->addCoordinateSystem(1.0);    // 添加坐标系，单位：m
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");    // 设置点大小

    // VTK 渲染窗口初始化
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();   // 创建 VTK 渲染窗口对象
    ui->openGLWidget->SetRenderWindow(renderWindow);  // 加载渲染对象
    viewer->setupInteractor(ui->openGLWidget->GetInteractor(), renderWindow);   // 设置交互器，如旋转缩放
    renderWindow->AddRenderer(viewer->getRendererCollection()->GetFirstRenderer()); // 将 viewer 的第一个渲染器添加到 VTK 渲染窗口中


    // 连接信号与槽
    connect(ui->loadPly, &QPushButton::clicked, this, &Widget::onLoadPlyclicked);
    connect(ui->loadMesh, &QPushButton::clicked, this, &Widget::onLoadMeshclicked);
    connect(ui->backgroundColor, &QPushButton::clicked, this, &Widget::changeBackgroundColor);
    connect(ui->pointColor, &QPushButton::clicked, this, &Widget::changePointCloudColor);
    connect(ui->restoreColor, &QPushButton::clicked, this, &Widget::restoreOriginalColor);
    connect(ui->pointSizeSlider, &QSlider::valueChanged, this, &Widget::onPointSizeChanged);

    viewer->registerPointPickingCallback(&Widget::pickPointCallback, *this);    // 点击 点云 调用回调函数
}
// 析构函数
Widget::~Widget() {
    if (viewer) {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
    }
    ui->openGLWidget->GetRenderWindow()->Finalize();
    ui->openGLWidget->GetRenderWindow()->Delete();
    delete ui;
}


// 槽函数：加载 PLY 点云文件
void Widget::onLoadPlyclicked()
{
    // 调试信息：按钮点击
    qDebug() << "Load PLY button clicked.";

    QString fileName = QFileDialog::getOpenFileName(this, "Open PLY", "..", "Open PLY files(*.ply)");

    // 如果没有选择文件，直接返回
    if (fileName.isEmpty())
        return;

    // 使用QTime来计算加载时间
    qDebug() << "Selected file: " << fileName;
    QTime loadTime;
    loadTime.start();

    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPLYFile(fileName.toStdString(), cloud_blob) == -1) {
        std::cerr << "Could not load file: " << fileName.toStdString() << std::endl;
        return;
    }
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);// 转换点云格式

    pcl::copyPointCloud(*cloud, *original_color_cloud);// 拷贝点云

    int chunkSize = defaultChunkSize;
    int total_points = cloud->points.size();
    num_chunks = (total_points + chunkSize - 1) / chunkSize; // 计算需要分多少块
    std::cout << "The number of chunk pointclouds is num_chunks : " << num_chunks << std::endl;
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    current_chunk = 0;
    while (current_chunk < num_chunks) {
        int startIdx = current_chunk * chunkSize;
        int endIdx = std::min(startIdx + chunkSize, (int)cloud->points.size());
        if (startIdx < endIdx) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr chunk(new pcl::PointCloud<pcl::PointXYZRGB>());
            chunk->points.assign(cloud->points.begin() + startIdx, cloud->points.begin() + endIdx);
            std::string cloud_id = "chunk_" + std::to_string(current_chunk);
            viewer->addPointCloud(chunk, cloud_id);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, currentPointSize, cloud_id);
            viewer->resetCamera();
            ui->openGLWidget->update();
            ui->openGLWidget->GetRenderWindow()->Render();
            std::cout << "Loaded chunk " << current_chunk << " with points from " << startIdx << " to " << endIdx << std::endl;
            current_chunk++;
        } else {
            std::cout << "Invalid chunk range: startIdx=" << startIdx << ", endIdx=" << endIdx << std::endl;
            current_chunk++;
        }
    }
    std::cout << "All chunks have been loaded." << std::endl;
    allChunksLoaded = true; // 设置所有块已加载

    // 输出加载文件时间
    int elapsed = loadTime.elapsed();
    qDebug() << "File loaded by view" << elapsed << "ms";
}

// 槽函数：加载 mesh 对象
void Widget::onLoadMeshclicked() {
    // 调试信息：按钮点击
    qDebug() << "Load MESH button clicked.";

    QString fileName = QFileDialog::getOpenFileName(this, "Open MESH", "..", "Open MESH files(*.ply *.obj *.stl)");
    if (fileName.isEmpty())
        return;

    // 使用QTime来计算加载时间
    qDebug() << "Selected file: " << fileName;
    QTime loadTime;
    loadTime.start();

    // 使用 PCL 加载 PolygonMesh 格式的文件
    if (pcl::io::loadPLYFile(fileName.toStdString(), *mesh) == -1) {
        std::cerr << "Could not load mesh file: " << fileName.toStdString() << std::endl;
        return;
    }
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPolygonMesh(*mesh, "mesh");
    viewer->resetCamera();

    int elapsed = loadTime.elapsed();
    qDebug() << "Mesh file loaded by view" << elapsed << "ms";

    ui->openGLWidget->update();
    ui->openGLWidget->GetRenderWindow()->Render();
}


// 槽函数：更改 视图背景 颜色
void Widget::changeBackgroundColor() {
    QColor color = QColorDialog::getColor(Qt::white, this, "Select Background Color");
    if (color.isValid()) {
        viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
        ui->openGLWidget->update();
        ui->openGLWidget->GetRenderWindow()->Render();
    }
}

// 槽函数：更改 点云 颜色
void Widget::changePointCloudColor() {
    QColor color = QColorDialog::getColor(Qt::white, this, "Select Point Cloud Color");
    if (!color.isValid()) {
        std::cout << "No color selected or invalid color." << std::endl;
        return;
    }
    if (color.isValid()) {
        // 遍历 cloud->points 中的每一个 pcl::PointXYZRGB 对象点，并通过引用更改它们
        int startIdx = 0;
        for (int i = 0; i < num_chunks; ++i) {
            int endIdx = std::min(startIdx + defaultChunkSize, (int)cloud->points.size());
            for (int j = startIdx; j < endIdx; ++j) {
                cloud->points[j].r = color.red();
                cloud->points[j].g = color.green();
                cloud->points[j].b = color.blue();
            }
            std::cout << "Updated color for chunk " << i << " from " << startIdx << " to " << endIdx - 1 << std::endl;
            startIdx = endIdx;
        }

        viewer->updatePointCloud(cloud, "cloud");
        viewer->removePointCloud("cloud");
        viewer->addPointCloud(cloud, "cloud");
        ui->openGLWidget->update();
        ui->openGLWidget->GetRenderWindow()->Render();
        std::cout << "Forced render update." << std::endl;
    }
}

// 槽函数：恢复 点云 颜色
void Widget::restoreOriginalColor() {
    pcl::copyPointCloud(*original_color_cloud, *cloud);
    viewer->updatePointCloud(cloud, "cloud");
    ui->openGLWidget->update();
    ui->openGLWidget->GetRenderWindow()->Render();
}

// 槽函数：捕捉 点 坐标 ，并显示两点距离
void Widget::pickPointCallback(const pcl::visualization::PointPickingEvent& event, void* args) {
    (void)args;  // 标记 args 为未使用
    if (event.getPointIndex() == -1)
        return;

    // 获取视图坐标中的点击点位置
    pcl::PointXYZRGB new_point;
    event.getPoint(new_point.x, new_point.y, new_point.z);

    // 将点击的点标记为红色
    new_point.r = 255;
    new_point.g = 0;
    new_point.b = 0;
    clicked_points_3d->points.push_back(new_point);

    // 在视图中显示红色的点击点
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(clicked_points_3d, 255, 0, 0);
    viewer->removePointCloud("clicked_points");
    viewer->addPointCloud(clicked_points_3d, red, "clicked_points");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

    std::cout << "Clicked Point Coordinates: " << new_point.x << " " << new_point.y << " " << new_point.z << std::endl;

    // 计算并显示两个点击点之间的距离
    if (clicked_points_3d->points.size() == 2) {
        const auto& p1 = clicked_points_3d->points[0];
        const auto& p2 = clicked_points_3d->points[1];

        double distance = std::sqrt(std::pow(p2.x - p1.x, 2) +
                                    std::pow(p2.y - p1.y, 2) +
                                    std::pow(p2.z - p1.z, 2));

        std::cout << "Distance between points: " << distance << std::endl;

        // 在两点之间画线
        viewer->removeShape("line");
        viewer->addLine<pcl::PointXYZRGB, pcl::PointXYZRGB>(p1, p2, 0.0, 1.0, 0.0, "line");

        // 显示距离文本
        std::string distance_text = "Distance: " + std::to_string(distance);
        pcl::PointXYZ mid_point;
        mid_point.x = (p1.x + p2.x) / 2.0;
        mid_point.y = (p1.y + p2.y) / 2.0;
        mid_point.z = (p1.z + p2.z) / 2.0;

        viewer->removeText3D("distance_text");
        viewer->addText3D(distance_text, mid_point, 0.2, 0.0, 1.0, 0.0, "distance_text");

        // 清除已选择的点
        clicked_points_3d->clear();
    }

    ui->openGLWidget->update();
    ui->openGLWidget->GetRenderWindow()->Render();
}


// 槽函数：更改点云大小
void Widget::onPointSizeChanged(int value) {
    // 更新全局变量存储当前点云大小
    currentPointSize = value;

    // 遍历并更新所有已加载的点云块
    for (int i = 0; i < num_chunks; i++) {
        std::string cloud_id = "chunk_" + std::to_string(i);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, cloud_id);
    }

    // 更新UI
    ui->openGLWidget->update();
    ui->openGLWidget->GetRenderWindow()->Render();
}



