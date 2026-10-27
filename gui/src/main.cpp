#include "mainwindow.h"

#include <QApplication>
#include <QFont>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setApplicationName(QStringLiteral("L103MotorControl"));

    /* 2026-10-23: 字太小看着累 —— 全局字号在系统默认基础上 +2 号。
     * 必须在建窗口之前设, 子控件才会继承。 */
    {
        QFont f = a.font();
        const int pt = (f.pointSize() > 0) ? f.pointSize() : 9;
        f.setPointSize(pt + 2);
        a.setFont(f);
    }

    MainWindow w;
    w.showMaximized();   /* 1080 高的屏上直接铺满, 页面内容再高也有滚动条 */

    return a.exec();
}
