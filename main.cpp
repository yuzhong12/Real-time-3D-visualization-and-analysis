/**
 * @file main.cpp
 * @author Kang Bingxiang， zhou jie
 * @date 2024-10-06
 *
 * @brief Implementation of QT+PCl+VTK.
 */

#include "widget.h"
#include <QApplication>

// 包含 消除vtk版本警告 头文件
#include <vtkOutputWindow.h>
#include <pcl/console/print.h>

// 包含 QVTKOpenGLNativeWidget 这个控件功能
#include <QSurfaceFormat>
#include "QVTKOpenGLNativeWidget.h"

int main(int argc, char *argv[])
{
    // 忽略 PCL 警告
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // 解决 OpenGL版本 太低问题
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication a(argc, argv);
    Widget w;
    w.show();
    return a.exec();
}
