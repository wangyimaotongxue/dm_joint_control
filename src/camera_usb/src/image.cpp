#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// 绘制瞄准器函数
void drawCrosshair(Mat& frame, const Point& center, int size, const Scalar& color, int thickness) {
    int halfSize = size / 2;
    
    // 绘制水平线
    line(frame, 
         Point(center.x - halfSize, center.y), 
         Point(center.x + halfSize, center.y), 
         color, thickness);
    
    // 绘制垂直线
    line(frame, 
         Point(center.x, center.y - halfSize), 
         Point(center.x, center.y + halfSize), 
         color, thickness);
    
    // 绘制中心圆点
    circle(frame, center, 3, color, -1);
    
    // 绘制外圈圆环
    circle(frame, center, size / 3, color, thickness);
}

// 绘制高级瞄准器函数
void drawAdvancedCrosshair(Mat& frame, const Point& center, int size, const Scalar& color, int thickness) {
    int halfSize = size / 2;
    int quarterSize = size / 4;
    
    // 主十字线
    line(frame, Point(center.x - halfSize, center.y), Point(center.x - quarterSize, center.y), color, thickness);
    line(frame, Point(center.x + quarterSize, center.y), Point(center.x + halfSize, center.y), color, thickness);
    line(frame, Point(center.x, center.y - halfSize), Point(center.x, center.y - quarterSize), color, thickness);
    line(frame, Point(center.x, center.y + quarterSize), Point(center.x, center.y + halfSize), color, thickness);
    
    // 内十字线（短）
    int shortSize = size / 6;
    line(frame, Point(center.x - shortSize, center.y), Point(center.x + shortSize, center.y), color, thickness);
    line(frame, Point(center.x, center.y - shortSize), Point(center.x, center.y + shortSize), color, thickness);
    
    // 中心圆点
    circle(frame, center, 2, color, -1);
    
    // 外圈圆环
    circle(frame, center, quarterSize, color, thickness);
    
    // 刻度标记
    int tickLength = 5;
    // 水平刻度
    for (int i = -halfSize + 20; i < halfSize; i += 20) {
        if (i != 0) {
            line(frame, 
                 Point(center.x + i, center.y - tickLength), 
                 Point(center.x + i, center.y + tickLength), 
                 color, 1);
        }
    }
    // 垂直刻度
    for (int i = -halfSize + 20; i < halfSize; i += 20) {
        if (i != 0) {
            line(frame, 
                 Point(center.x - tickLength, center.y + i), 
                 Point(center.x + tickLength, center.y + i), 
                 color, 1);
        }
    }
}

int main() {
    // 打开默认摄像头（索引0）
    VideoCapture cap(0);
    
    if (!cap.isOpened()) {
        cout << "无法打开摄像头" << endl;
        return -1;
    }
    
    // 设置摄像头参数（可选）
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 30);
    
    cout << "摄像头参数：" << endl;
    cout << "宽度: " << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
    cout << "高度: " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "帧率: " << cap.get(CAP_PROP_FPS) << endl;
    
    Mat frame;
    
    while (true) {
        cap >> frame;
        
        if (frame.empty()) {
            cout << "无法获取帧" << endl;
            break;
        }
        
        // 计算帧中心点
        Point center(frame.cols / 2, frame.rows / 2);
        
        // 绘制瞄准器 - 选择其中一种样式
        
        // 样式1: 简单十字准星
        drawCrosshair(frame, center, 50, Scalar(0, 255, 0), 2);
        
        // 样式2: 高级瞄准器（取消注释使用）
        // drawAdvancedCrosshair(frame, center, 100, Scalar(0, 0, 255), 2);
        
        // 样式3: 红色点状准星（取消注释使用）
        // circle(frame, center, 5, Scalar(0, 0, 255), -1);
        // circle(frame, center, 15, Scalar(0, 0, 255), 2);
        
        // 显示中心坐标
        string coord_text = "Center: (" + to_string(center.x) + ", " + to_string(center.y) + ")";
        putText(frame, coord_text, Point(10, frame.rows - 40), 
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
        
        // 在帧上添加文字信息
        putText(frame, "Press q to exit", Point(3, 20), 
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        
        namedWindow("Camera with Crosshair", WINDOW_NORMAL);
        imshow("Camera with Crosshair", frame);
        
        if (waitKey(0) == 113) {
            break;
        }
    }
    
    cap.release();
    destroyAllWindows();
    
    return 0;
}