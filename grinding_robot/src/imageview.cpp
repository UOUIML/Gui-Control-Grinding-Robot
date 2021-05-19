#include "imageview.h"

ImageView::ImageView(QWidget *parent):QLabel(parent)
{
	_curPath = QDir::currentPath();
	_curPath = QDir::toNativeSeparators(_curPath);
}

bool ImageView::LoadImageMat(QString &name){

    try
        {
            this->_img = cv::imread(name.toStdString());
			if (!_img.empty()) qDebug() << "Image Okay";
            if (_img.empty()) return false;
            //_Qimg = QImage((uchar*)_img.data, _img.cols, _img.rows, QImage::Format_RGB888);
            _Qimg.load(name);
			if (_Qimg.isNull()) qDebug() << "Qimage not Okay";

            QPixmap pixmap = QPixmap::fromImage(_Qimg);

            int w = width();
            int h = height();

            setPixmap(pixmap.scaled(w, h, Qt::IgnoreAspectRatio));

			// clear when is done
			//_pixmap = QPixmap::fromImage(_Qimg);

		
            show();
		}
        catch (std::exception& e) {
            qDebug() << e.what();
        }

        return true;

}

bool ImageView::setImageMat(cv::Mat & img)
{
	try
	{
		this->_img = img.clone();
		if (_img.empty()) return false;
		QString name = "thocao.jpg";
		cv::imwrite(name.toStdString(), _img);
		//_Qimg = QImage((uchar*)temp.data, temp.cols, temp.rows, QImage::Format_RGB888);
		QString fullPathName = _curPath +"/"+ name;
		_Qimg.load(fullPathName);
		//qDebug() << fullPathName;

		QPixmap pixmap = QPixmap::fromImage(_Qimg);

		int w = width();
		int h = height();

		setPixmap(pixmap.scaled(w, h, Qt::IgnoreAspectRatio));
		show();

	}
	catch (std::exception& e) {
		qDebug() << e.what();
	}

	return true;
}

bool ImageView::setImageMatRoi(cv::Mat & img)
{
	try
	{
		this->_img = img.clone();
		if (_img.empty()) return false;
		QString name = "thocao.jpg";
		cv::imwrite(name.toStdString(), _img);
		//_Qimg = QImage((uchar*)temp.data, temp.cols, temp.rows, QImage::Format_RGB888);
		_Qimg.load(name);

		QPixmap pixmap = QPixmap::fromImage(_Qimg);

		int w = width();
		int h = height();

		setPixmap(pixmap.scaled(w, h, Qt::IgnoreAspectRatio));
		show();
		this->_rec = cv::Rect();

		_pixmap = QPixmap::fromImage(_Qimg);
	}
	catch (std::exception& e) {
		qDebug() << e.what();
	}

	return true;
}

bool ImageView::GetMatandRoi(cv::Mat & img, cv::Rect & roi)
{
	if (this->_img.empty() || this->_rec.empty()) return false;

	img = this->_img.clone();
	roi = this->_rec;
	return true;
}

cv::Mat ImageView::GetRoiMat(cv::Rect &rect)
{
    if(this->_img.empty()) return cv::Mat();
    cv::Mat roi = _img(rect);
    return roi.clone();
}

cv::Mat ImageView::GetRoiMat()
{
    if(this->_img.empty() || this->_rec.empty()) return cv::Mat();

    cv::Mat roi = _img(_rec);
    return roi.clone();

}

cv::Mat ImageView::GetOriginalMat()
{
	if (this->_img.empty()) return  cv::Mat();

	return this->_img;
}

cv::Point ImageView::GetImageXY()
{
	return _imgXY;
}

void ImageView::DrawRect(QPoint &startPoint, QPoint &endPoint)
{
    QPixmap pixmap = QPixmap::fromImage(_Qimg);

    int W = this->width();
    int H = this->height();

    _scaleX = (float) _img.cols/W;
    _scaleY =  (float)_img.rows/H;

    int ww = (float)abs(startPoint.x() - endPoint.x())*_scaleX;
    int hh = (float)abs(startPoint.y() - endPoint.y())*_scaleY;

    _rec = cv::Rect(((int)startPoint.x()*_scaleX),(int)startPoint.y()*_scaleY,ww,hh);

	this->_imgXY.x = (int)startPoint.x()*_scaleX;
	this->_imgXY.y = (int)(int)startPoint.y()*_scaleY;


    qDebug()<<"x: "<<_rec.x <<" y: "<<_rec.y<<" w: "<<_rec.width<<" h: "<<_rec.height;

    pixmap =  pixmap.scaled(W,H,Qt::IgnoreAspectRatio);

    QRectF recQ(startPoint,endPoint);

    QPainter painter(&pixmap);
    QPen Red(Qt::red,3);
    painter.setPen(Red);

    painter.drawRect(recQ);
    setPixmap(pixmap);

	show();
}

cv::Point2f ImageView::DrawMaker(QPoint & centerPoint)
{
	cv::Point2f outputPoint;

	int W = this->width();
	int H = this->height();

	_scaleX = (float)_img.cols / W;
	_scaleY = (float)_img.rows / H;

	outputPoint.x = _scaleX*centerPoint.x();
	outputPoint.y = _scaleY*centerPoint.y();

	_pixmap = _pixmap.scaled(W, H, Qt::IgnoreAspectRatio);

	QPainter painter(&_pixmap);
	QPen Red(Qt::red,3);
        
	painter.setPen(Red);

	//painter;
	painter.drawPoint(centerPoint);
	setPixmap(_pixmap);

	show();

	return outputPoint;
}

cv::Point2f ImageView::DrawMakerV2(QPoint & centerPoint)
{
	QPixmap pixmap = QPixmap::fromImage(_Qimg);

	cv::Point2f outputPoint;

	int W = this->width();
	int H = this->height();

	_scaleX = (float)_img.cols / W;
	_scaleY = (float)_img.rows / H;

	outputPoint.x = _scaleX * centerPoint.x();
	outputPoint.y = _scaleY * centerPoint.y();

	pixmap = pixmap.scaled(W, H, Qt::IgnoreAspectRatio);

	QPainter painter(&pixmap);
	QPen Red(Qt::red,3);
	painter.setPen(Red);

	//painter;
	painter.drawPoint(centerPoint);
	setPixmap(pixmap);

	show();

	return outputPoint;
}

QImage ImageView::GetQImage()
{
     if(this->_img.empty()) return QImage();
    return _Qimg;
}


void ImageView::GetRatio(float& ratioX,float& ratioY)
{
     if(this->_img.empty()) return;
     int W = this->width();
     int H = this->height();
     ratioX = (float) _img.cols/W;
     ratioY = (float) _img.rows/H;

     _scaleX = ratioX;
     _scaleY = ratioY;

}

