#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H

#include <QObject>
#include <QLabel>
#include <QPainter>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <qdir.h>

class ImageView: public QLabel
{
     Q_OBJECT
public:
    ImageView(QWidget *parent);

    bool LoadImageMat(QString& name);
	bool setImageMat(cv::Mat& img);
	bool setImageMatRoi(cv::Mat& img);
	bool GetMatandRoi(cv::Mat& img, cv::Rect& roi);
    cv::Mat GetRoiMat(cv::Rect& rect);
    cv::Mat GetRoiMat();
	cv::Mat GetOriginalMat();
	cv::Point GetImageXY();
    void DrawRect(QPoint& startPoint, QPoint& endPoint);
	cv::Point2f DrawMaker(QPoint& centerPoint);
	cv::Point2f DrawMakerV2(QPoint& centerPoint);


    QImage GetQImage();

    void GetRatio(float& ratioX,float& ratioY);

protected:

private:
    cv::Mat _img;
    QImage  _Qimg;
    cv::Rect _rec;
    float _scaleX,_scaleY;

	cv::Point _imgXY;

	QPixmap _pixmap;

	QString _curPath;

};

#endif // IMAGEVIEW_H

