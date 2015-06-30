/******************************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
tamp(const sensor_msgs::header& msg);

*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: 
*   Protopapas Marios <protopapas_marios@hotmail.com>
*   Manos Tsardoulias <etsardou@gmail.com>
*********************************************************************/

#include "pandora_vision_annotator/annotator_connector.h"

namespace pandora_vision
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CConnector::CConnector(int argc, char **argv):
    QObject(),
    loader_(argc, argv),
    argc_(argc),
    argv_(argv)
  {
    // Event filter for the image area
    loader_.imageLabel->installEventFilter(this);
    //loader_.scrollArea->installEventFilter(this);
    for (int i = 0; i < 5; i++)
    {
        bbox_ready.push_back(0);
    }

    img_state_ = IDLE;

    package_path = ros::package::getPath("pandora_vision_annotator");
    
    QObject::connect(
      loader_.offlineRadioButton, SIGNAL(toggled(bool)),
      this, SLOT(offlineRadioButtonChecked()));
    QObject::connect(
      loader_.onlineRadioButton, SIGNAL(toggled(bool)),
      this, SLOT(onlineRadioButtonChecked()));
  }

  /**
  @brief Qt slot that is called when the offlineRadioButton is checked
  @return void
  **/
  void CConnector::offlineRadioButtonChecked(void)
  {  
    ROS_INFO("offline checked");
    QObject::connect(
        loader_.appendCheckBox, SIGNAL(toggled(bool)),
        this, SLOT(appendCheckBoxChecked()));
    QObject::connect(
        loader_.removeFilePushButton, SIGNAL(clicked(bool)),
        this, SLOT(removeFilePushButtonTriggered()));
    QObject::connect(
        loader_.saveImagesPushButton, SIGNAL(clicked(bool)),
        this, SLOT(saveImagesPushButtonTriggered()));
    QObject::connect(
      loader_.rosTopicPushButton, SIGNAL(clicked(bool)),
      this, SLOT(rosTopicPushButtonTriggered()));
    QObject::connect(
        loader_.victimPushButton, SIGNAL(clicked(bool)),
        this, SLOT(victimPushButtonTriggered()));
    QObject::connect(
        loader_.holePushButton, SIGNAL(clicked(bool)),
        this, SLOT(holePushButtonTriggered()));
    QObject::connect(
        loader_.hazmatPushButton, SIGNAL(clicked(bool)),
        this, SLOT(hazmatPushButtonTriggered()));
    QObject::connect(
        loader_.landoltcPushButton, SIGNAL(clicked(bool)),
        this, SLOT(landoltcPushButtonTriggered()));
    QObject::connect(
        loader_.qrPushButton, SIGNAL(clicked(bool)),
        this, SLOT(qrPushButtonTriggered()));
    QObject::connect(
        loader_.submitPushButton, SIGNAL(clicked(bool)),
        this, SLOT(submitPushButtonTriggered()));
    QObject::connect(
        loader_.clearPushButton, SIGNAL(clicked(bool)),
        this, SLOT(clearPushButtonTriggered()));
    QObject::connect(
        loader_.nextFramePushButton, SIGNAL(clicked(bool)),
        this, SLOT(nextFramePushButtonTriggered()));
    QObject::connect(
        loader_.previousFramePushButton, SIGNAL(clicked(bool)),
        this, SLOT(previousFramePushButtonTriggered()));
    QObject::connect(
        loader_.predatorPushButton, SIGNAL(clicked(bool)),
        this, SLOT(predatorPushButtonTriggered()));
    QObject::connect(
        loader_.framesListWidget, SIGNAL(itemClicked(QListWidgetItem*)),
        this, SLOT(frameLabelTriggered(QListWidgetItem*)));
    Q_EMIT offlineModeGiven();
    state_= OFFLINE;
  }
  /**
  @brief Qt slot that is called when the onlineRadioButton is checked
  @return void
  **/
  void CConnector::onlineRadioButtonChecked(void)
  {
    QObject::connect(
      loader_.rosTopicPushButton, SIGNAL(clicked(bool)),
      this, SLOT(rosTopicPushButtonTriggered()));
       Q_EMIT onlineModeGiven();
      state_ = ONLINE;
  }
  /**
  @brief Qt slot that is called when the appendCheckBOx is checked
  @return void
  **/
  void CConnector::appendCheckBoxChecked(void)
  {
    ROS_INFO("APPEND CHECKED");
    Q_EMIT appendToFile();
  }

  /**
  @brief Qt slot that is called when the rosTopicPushButton is pressed
  @return void
  **/
  void CConnector::rosTopicPushButtonTriggered(void)
  {    Q_EMIT rosTopicGiven();
       
  }

  /**
  @brief Qt slot that is called when the removeFilePushButton is pressed
  @return void
  **/
  void CConnector::removeFilePushButtonTriggered(void)
  {
    Q_EMIT removeFile();
  }

 /**
  @brief Qt slot that is called when the saveImagesPushButton is pressed
  @return void
  **/
  void CConnector::saveImagesPushButtonTriggered(void)
  {
     Q_EMIT saveImages();
  }



  /**
  @brief Qt slot that is called when the predatorPushButton is pressed
  @return void
  **/
  void CConnector::predatorPushButtonTriggered(void)
  { 
    if(ImgAnnotations::annotations.size() != 0)
    {
      Q_EMIT predatorEnabled();
      state_ = PREDATOR;
      std::stringstream file;
      file << package_path << "/data/annotations.txt";
    }
    else
      setFullFrameAnnotations();
  }
  /**
  @brief function that calls the show() function of loader in order to
  load the ui
  @return void
  **/
  void CConnector::show(void)
  {
    loader_.show();
  }
  
  /**
  @brief function that gets the rostopic given 
  by the user
  @return void
  **/
  QString CConnector::getRosTopic(void)
  {
    return loader_.rosTopicLineEdit->text();
  }
  
  /**
  @brief function that gets the bag name given 
  by the user
  @return void
  **/
  QString CConnector::getBagName(void)
  {
    return loader_.bagLineEdit->text();
  }
  
  /**
  @brief function that sets the frames loaded from bag
  @param x [const std::vector<cv::Mat>&] the vector of frames
  @param offset [int] the offset to be used for numbering
  @return void
  **/
  void CConnector::setFrames(const std::vector<cv::Mat>& x, int offset)
  {
      frames = x;
      std::cout << frames.size();
      offset_ = offset;
      QString label;
      for(int i = offset_; i < frames.size()+offset_; i++)
      {
        label = QString("frame %1").arg(i);
        loader_.framesListWidget->addItem(label);
      }
  }

  /**
  @brief function that sets the current frame for display
  transforming it into QImage
  @param x [int] the current frame
  @return void
  **/
  void CConnector::setcurrentFrame(int x)
  {
    currFrame = x;
    QImage dest((const uchar *) frames[currFrame].data, frames[currFrame].cols,
                frames[currFrame].rows, frames[currFrame].step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation
    setImage(dest);
    updateImage();
  } 
  
  /**
  @brief function that sets the current image state
  @param state [std::string] the current state
  @return void
  **/
  void CConnector::setState(pandora_vision::States state)
  {
    state_ = state;
  }

  /**
  @brief function that returns the current Frame
  @param x [int] the current frame number
  @ param frame [cv::Mat*] the actual frame matrix
  @return void
  **/
  void CConnector:: getcurrentFrame(int x, cv::Mat* frame)
  {
    *frame = frames[currFrame];
  }
  
  /**
  @brief function that returns the current Frame
  @return [int] the frame number
  **/
  int CConnector::getFrameNumber()
  {
    return  currFrame;
  }
  
  /**
  @brief function that sets the QIMage
  @param img [Qimage] the QImage
  @return void
  **/
  void CConnector::setImage(const QImage& img)
  {
       localImage_ = img.copy();
       if(currFrame == offset_)
          ImgAnnotations::setOriginalImgDimensions(localImage_.width(), localImage_.height());
       localImage_ = localImage_.scaled(640, 480,  Qt::KeepAspectRatio, Qt::SmoothTransformation);
  }
 
  /**
  @brief Qt slot when its called updates the Qpixmap for display
  @return void
  **/
  void CConnector::updateImage()
  {
    loader_.imageLabel->setPixmap(QPixmap::fromImage(localImage_));
    // loader_.imageLabel->setScaledContents(true);
  }
  
  /**
  @brief function that sets the msg Header from bag
  @param msg [const std_msgs::Header&] the msg Header
  @return void
  **/
  void CConnector::msgTimeStamp(const std_msgs::Header& msg)
  {
    msgHeader = msg;
  }
  
  /**
  @brief function that sets the msg from bag
  @param msg [const sensor_msgs::ImageConstPtr&] the msg Header
  @return void
  **/
  void CConnector::setMsg(const sensor_msgs::ImageConstPtr& msg)
  {
  msg_ = *msg;
  }
  
  /**
  @brief Transform QImage to cv::Mat
  @param src [QImage const&] The QImage
  @return cv::Mat
  **/
  cv::Mat CConnector::QImage2Mat(QImage const& src)
  {
     cv::Mat tmp(src.height(), src.width(), CV_8UC3, const_cast<uchar*>(src.bits()), src.bytesPerLine());
     cv::Mat result; 
     cvtColor(tmp, result, CV_BGR2RGB);
     return result;
  }

  /**
  @brief General event filter. Captures all events
  @param watched [QObject*] The object in which the event was triggered
  @param event [QEvent*] The type of event
  @return bool : True is event served
  **/
  bool CConnector::eventFilter( QObject* watched, QEvent* event )
  {
    if(watched == loader_.imageLabel)
    {
      if(event -> type() == QEvent::KeyPress)
      {
          QKeyEvent*  me = static_cast<QKeyEvent*> (event);
          if(me->key() == Qt::Key_S)
          {
            //qDebug("Save current Frame as: frame%d.png",currFrame);
            if(state_== ONLINE)
            {
              //do
              //{
              std::stringstream img_name, file;
              img_name << package_path << "/data/frame" << msgHeader.seq <<".png";
              //cv::Mat temp = QImage2Mat(localImage_);
              //cv::imwrite(img_name.str(),temp);
              //loader_.statusLabel->setText("Save current Frame as:" + QString(img_name.str().c_str() )); 
              file << package_path << "/data/onlineAnnotations.txt";
              ImgAnnotations::writeToFile(file.str(), msgHeader);
              loader_.statusLabel->setText("Writing to file"+QString(file.str().c_str()));
              //}while(me->key() != Qt::Key_D);
            }

            if(state_ == OFFLINE)
            {
              std::stringstream img_name;
              img_name << package_path  << "/data/frame" << currFrame + offset_ << ".png";
              cv::Mat temp;
              cv::cvtColor(frames[currFrame], temp, CV_BGR2RGB);
              cv::imwrite(img_name.str(), temp);
              loader_.statusLabel->setText("Save current Frame as:" + QString(img_name.str().c_str() )); 
            }
          }
      }

      if(event->type() == QEvent::MouseButtonPress && state_ != PREDATOR )
      {
        loader_.imageLabel->setFocus(Qt::MouseFocusReason);
        std::string img_name = "frame" + boost::to_string(currFrame + offset_) + ".png";
        const QMouseEvent* const me =
          static_cast<const QMouseEvent*>( event );
        QPoint p = me->pos();


        int container_width = loader_.imageLabel->width();
        int container_height = loader_.imageLabel->height();
        int img_height = localImage_.height();
        int diff = (container_height - img_height) / 2;

        if(me->button() == Qt::LeftButton)
        {
          switch (img_state_)
          {
              case IDLE:
                  {
                      return true;
                  }
              case VICTIM_CLICK:
                  {
                    loader_.victimCoordsLabel->setText(
                     loader_.victimCoordsLabel->text() + QString("[") +
                     QString().setNum(p.x()) + QString(",") +
                     QString().setNum(p.y() - diff) + QString("]"));
                     ImgAnnotations::setAnnotations(img_name, "1", p.x(), p.y()-diff);
                    bbox_ready[0]++;
                    if(bbox_ready[0] == 2)
                    {
                        drawBox();
                        bbox_ready[0] = 0;
                        ImgAnnotations::annPerImage++;
                    }
                    break;
                  }
              case HOLE_CLICK:
                  {
                    loader_.holeCoordsLabel->setText(
                     loader_.holeCoordsLabel->text() + QString("[") +
                     QString().setNum(p.x()) + QString(",") +
                     QString().setNum(p.y() - diff) + QString("]"));
                     ImgAnnotations::setAnnotations(img_name, "-1", p.x(), p.y()-diff);
                    bbox_ready[0]++;
                    if(bbox_ready[0] == 2)
                    {
                      drawBox();
                      bbox_ready[0] = 0;
                      ImgAnnotations::annPerImage++;
                    }
                    break;
                  }

              case QR_CLICK:
                  {
                    loader_.qrCoordsLabel->setText(
                     loader_.qrCoordsLabel->text() + QString("[") +
                     QString().setNum(p.x()) + QString(",") +
                     QString().setNum(p.y() - diff) + QString("]"));
                    ImgAnnotations::setAnnotations(img_name, "QR", p.x(), p.y()-diff);
                    bbox_ready[1]++;
                    if(bbox_ready[1] == 2)
                    {
                        drawBox();
                        bbox_ready[1] = 0;
                        ImgAnnotations::annPerImage++;
                    }
                    break;
                  }
               case HAZMAT_CLICK:
                  {
                    loader_.hazmatCoordsLabel->setText(
                    loader_.hazmatCoordsLabel->text() + QString("[") +
                     QString().setNum(p.x()) + QString(",") +
                     QString().setNum(p.y() - diff) + QString("]"));
                    std::string type =loader_.hazmatInfoLineEdit->text().toStdString();
                    ImgAnnotations::setAnnotations(img_name, "Hazmat", p.x(), p.y()-diff, type);
                    bbox_ready[2]++;
                    if(bbox_ready[2] == 2)
                    {
                        drawBox();
                        bbox_ready[2] = 0;
                        ImgAnnotations::annPerImage++;
                    }
                    break;
                  }
               case LANDOLTC_CLICK:
                  {
                    loader_.landoltcCoordsLabel->setText(
                     loader_.landoltcCoordsLabel->text() + QString("[") +
                     QString().setNum(p.x()) + QString(",") +
                     QString().setNum(p.y() - diff) + QString("]"));
                    ImgAnnotations::setAnnotations(img_name, "LandoltC", p.x(), p.y()-diff);
                    bbox_ready[3]++;
                    if(bbox_ready[3] == 2)
                    {
                        drawBox();
                        bbox_ready[3] = 0;
                        ImgAnnotations::annPerImage++;
                    }
                    break;
                  }
          }
        }
      }
    }
    return false;
  }
  
  /**
  @brief Qt slot that is called when the victim pushButton 
  is triggered. Sets the image State.
  @return void
  **/
  void CConnector::victimPushButtonTriggered(void)
  { 
    if(bbox_ready[0]== 0)
    img_state_ = VICTIM_CLICK;
  }
  
  /**
  @brief Qt slot that is called when the holePushButton 
  is triggered. Sets the image State.
  @return void
  **/
  void CConnector::holePushButtonTriggered(void)
  { 
    if(bbox_ready[1]== 0)
    img_state_ = HOLE_CLICK;
  }
  
  /**
  @brief Qt slot that is called when the qrPushButton 
  is triggered. Sets the image State.
  @return void
  **/
  void CConnector::qrPushButtonTriggered(void)
  {
    if(bbox_ready[2]== 0)
    img_state_ = QR_CLICK;
  }
 
  /**
  @brief Qt slot that is called when the hazmatPushButton 
  is triggered. Sets the image State.
  @return void
  **/
  void CConnector::hazmatPushButtonTriggered(void)
  {
    if(bbox_ready[3]== 0)
    img_state_ = HAZMAT_CLICK;
  }
 
  /**
  @brief Qt slot that is called when the landoltcPushButton 
  is triggered. Sets the image State.
  @return void
  **/
  void CConnector::landoltcPushButtonTriggered(void)
  {
    if(bbox_ready[4] == 0)
    img_state_ = LANDOLTC_CLICK;
  }
  
  /**
  @brief Qt slot that is called when the submitPushButton 
  is triggered. Saves current frame and annotations to file.
  @return void
  **/
  void CConnector::submitPushButtonTriggered(void)
  { 
    std::stringstream file;
    file << package_path << "/data/annotations.txt";
    if(ImgAnnotations::is_file_exist(file.str().c_str()) && currFrame == offset_)
    {
      remove(file.str().c_str());
    }
    ImgAnnotations::writeToFile(file.str() );
    std::stringstream img_name;
    img_name << package_path  << "/data/frame" << currFrame + offset_<< ".png";
    cv::Mat temp;
    cv::cvtColor(frames[currFrame], temp, CV_BGR2RGB);
    cv::imwrite(img_name.str(), temp);
    loader_.statusLabel->setText("Save current Frame as:" + QString(img_name.str().c_str() )); 
  }
   

  /**
  @brief Qt slot that is called when the clearPushButton 
  is triggered.delete annotations from current frame and file.
  @return void
  **/
  void CConnector::clearPushButtonTriggered(void)
  {
    std::stringstream file;
    file << package_path << "/data/annotations.txt";
    std::string img_name = "frame" + boost::to_string(currFrame+offset_) + ".png";
    img_state_= IDLE;
    bbox_ready.clear();
    loader_.landoltcCoordsLabel->clear();
    loader_.qrCoordsLabel->clear();
    loader_.hazmatCoordsLabel->clear();
    loader_.victimCoordsLabel->clear();
    loader_.statusLabel->clear();
    ImgAnnotations::annPerImage = 0;
    ImgAnnotations::annotations.clear();
    ImgAnnotations::deleteFromFile(file.str(), img_name);
    loader_.statusLabel->setText("Delete Annotations from " +  QString(img_name.c_str()));
    setcurrentFrame(currFrame);
  }
  
  /**
  @brief Qt slot that is called when the nextFramePushButton 
  is triggered.display next frame and load annotations from file.
  @return void
  **/
  void CConnector::nextFramePushButtonTriggered(void)
  {
    if(currFrame != frames.size()-1)
    {
      std::stringstream file;
      file << package_path << "/data/annotations.txt";
      currFrame++;
      setcurrentFrame(currFrame);
      std::string img_name = "frame" + boost::to_string(currFrame+offset_) + ".png";
      loader_.statusLabel->setText(QString(img_name.c_str()));
      ImgAnnotations::annotations.clear();
      ImgAnnotations::readFromFile(file.str(), img_name);
      ImgAnnotations::annPerImage = ImgAnnotations::annotations.size();
      loader_.statusLabel->setText("Loading " + QString().setNum(ImgAnnotations::annPerImage) + 
                                   " for " + QString(img_name.c_str()));
      if(ImgAnnotations::annPerImage !=0)
      drawBox();
    }
  }
 
  /**
  @brief Qt slot that is called when the previousFramePushButton 
  is triggered.display previous frame and load annotations from file.
  @return void
  **/
  void CConnector::previousFramePushButtonTriggered(void)
  {
    if(currFrame  != 0)
    {
      std::stringstream file;
      file << package_path << "/data/annotations.txt";
      currFrame--;
      setcurrentFrame(currFrame);
      std::string img_name = "frame" + boost::to_string(currFrame + offset_)+ ".png";
      ImgAnnotations::annotations.clear();
      ImgAnnotations::readFromFile(file.str(), img_name);
      ImgAnnotations::annPerImage = ImgAnnotations::annotations.size();
      loader_.statusLabel->setText("Loading " +QString().setNum(ImgAnnotations::annPerImage) +
                                   " for " + QString(img_name.c_str()));
      if(ImgAnnotations::annPerImage != 0)
      drawBox();
    }
  }
  
  /**
  @brief function that when is called saves full
  frame annotations for all images
  @return void
  **/
  void CConnector::setFullFrameAnnotations(void)
  {
    while(currFrame <= frames.size()-1)
    {
      ImgAnnotations::annotations.clear();
      std::string img_name = "frame" + boost::to_string(currFrame + offset_) + ".png";
      std::string category;
      /*if(img_state_ == VICTIM_CLICK)
        category = "1";
      if(img_state_ == HOLE_CLICK)*/
        category = "-1";
      ImgAnnotations::setAnnotations(img_name, category, 0, 0);
      ImgAnnotations::setAnnotations(img_name, category, 640, 480);
      //drawBox();  
      std::stringstream file;
      file << package_path << "/data/annotations.txt";
      ImgAnnotations::writeToFile(file.str() );
      std::stringstream imgName;
      imgName << package_path  << "/data/frame" << currFrame + offset_ << ".png";
      cv::Mat temp;
      cv::cvtColor(frames[currFrame], temp, CV_BGR2RGB);
      cv::imwrite(imgName.str(), temp);
      loader_.statusLabel->setText("Save current Frame as:" + QString(imgName.str().c_str() ));
      nextFramePushButtonTriggered();
      if(currFrame == frames.size()-1)
        break;
    }
  }
  
  /**
  @brief Qt slot that is called when a QListWidgetItem
  is triggered.display frame and load annotations from file.
  @return void
  **/
  void CConnector::frameLabelTriggered(QListWidgetItem* item)
  {
    QStringList temp = item->text().split(" ");
    std::stringstream file;
    file << package_path << "/data/annotations.txt";
    currFrame = temp.at(1).toInt() - offset_;
      setcurrentFrame(currFrame);
      std::string img_name = "frame" + boost::to_string(currFrame + offset_ )+ ".png";
      ImgAnnotations::annotations.clear();
      ImgAnnotations::readFromFile(file.str(), img_name);
      ImgAnnotations::annPerImage = ImgAnnotations::annotations.size();
      loader_.statusLabel->setText("Loading " +QString().setNum(ImgAnnotations::annPerImage) +
                                   " for " + QString(img_name.c_str()));
      if(ImgAnnotations::annPerImage !=0)
      drawBox();
  }

  void CConnector::setPredatorValues(int x, int y, int width, int height, bool initial )
  {
     if(ImgAnnotations::annotations[0].x1 == x && ImgAnnotations::annotations[0].y1 == y &&
        ImgAnnotations::annotations[0].x2 == x + width && ImgAnnotations::annotations[0].y2 == y + height && !initial)
     {
       ROS_INFO_STREAM("SAME ANNOTATION AS PREVIOUS FRAME");
     }
     else
     {
       ImgAnnotations::annotations.clear();
       std::string img_name = "frame" + boost::to_string(currFrame + offset_) + ".png";
       std::string category;
       if(img_state_ == VICTIM_CLICK)
         category = "1";
       if(img_state_ == HOLE_CLICK)
         category = "-1";
       ImgAnnotations::setAnnotations(img_name, category, x, y);
       ImgAnnotations::setAnnotations(img_name, category, x+width, y+height);
       drawBox();  
       std::stringstream file;
       file << package_path << "/data/annotations.txt";
       ImgAnnotations::writeToFile(file.str() );
       std::stringstream imgName;
       imgName << package_path  << "/data/frame" << currFrame + offset_ << ".png";
       cv::Mat temp;
       cv::cvtColor(frames[currFrame], temp, CV_BGR2RGB);
       cv::imwrite(imgName.str(), temp);
       loader_.statusLabel->setText("Save current Frame as:" + QString(imgName.str().c_str() )); 
    }
  }
  
  /**
  @brief function that draws annotations to the 
  current QImage
  @return void
  **/
  void CConnector::drawBox()
  {
    QPainter painter(&localImage_);
    painter.setRenderHint(QPainter::Antialiasing);
    QPen pen;
    pen.setWidth( 3 );
    pen.setBrush(Qt::green);
    painter.setPen(pen);
    for (int i = 0; i < ImgAnnotations::annotations.size(); i++)
    {
      loader_.statusLabel->setText("drawing "+ QString().setNum(i+1)+" annotation\n"
                                 +QString().setNum(ImgAnnotations::annotations[i].x1)+"," 
                                 +QString().setNum(ImgAnnotations::annotations[i].y1)+"," 
                                 +QString().setNum(ImgAnnotations::annotations[i].x2)+","
                                 +QString().setNum(ImgAnnotations::annotations[i].y2));
      QPoint p1(ImgAnnotations::annotations[i].x1,
                ImgAnnotations::annotations[i].y1);
      QPoint p2(ImgAnnotations::annotations[i].x2,
                ImgAnnotations::annotations[i].y2);
      QRect r(p1, p2);
      painter.drawRect(r);
      Q_EMIT updateImage();
    }
  }
}// namespace pandora_vision
