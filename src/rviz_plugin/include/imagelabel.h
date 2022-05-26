#ifndef IMAGELABEL_H
#define IMAGELABEL_H

#include <QLabel>
#include <QPoint>
#include <Qt>
#include <QMouseEvent>
#include <QPixmap>
#include <QPainter>

class ImageLabel : public QLabel{
Q_OBJECT
public:
    ImageLabel(QWidget* parent = Q_NULLPTR);
    ~ImageLabel();
    void mousePressEvent(QMouseEvent* event);
    void drawPixmap(QImage img);
    QPoint** getBoatSide();
    QPoint** getBoatSide2();
    QPoint** getBoatSide3();
    int getCritLine(int);
    int getCritLine2(int);
    void changeControlPoint(int x, int y);
    void controlBalls(int xr, int xm, int ym, int xh, int yh);
    void changeCritLine(int, int);
    void changeCritLine2(int, int);

    QImage* getImageCamera();
    // void paintEvent(QPaintEvent*);

private:
    enum {
        CRITLINE = 1, CRITLINE_2=2, LEFTBOAT = 3, RIGHTBOAT = 4, LEFTBOAT_2 = 5, RIGHTBOAT_2 = 6 , LEFTBOAT_3=7 , RIGHTBOAT_3=8
    };

    const int Width = 640, Height = 480;
    QPoint controlPoint, leftBoat_top, leftBoat_bottom, rightBoat_top, rightBoat_bottom;
    QPoint leftBoat_top2, leftBoat_bottom2, rightBoat_top2, rightBoat_bottom2;
    QPoint leftBoat_top3, leftBoat_bottom3, rightBoat_top3, rightBoat_bottom3;
    QPoint* boatSide[4];
    QPoint* boatSide2[4];
    QPoint* boatSide3[4];
    QPoint redBall, greenBall, redFar, greenFar,redMid,greenMid;
    int critLine, horizon,critLine2;
    int setMode;
    QImage *img_saved;
    QImage img_to_send;


public Q_SLOTS:
            void changeSetMode(int);

Q_SIGNALS:
            void sendCameraImg(QImage*);


};

#endif // IMAGELABEL_H