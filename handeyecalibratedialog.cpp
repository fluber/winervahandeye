#include "handeyecalibratedialog.h"
#include "ui_handeyecalibratedialog.h"
#include "hiwinrobot.h"
#include <QRegularExpression>
#include <QListWidgetItem>
#include "nxlibtransformationhelper.h"
#include <QTime>
#include <QMessageBox>
#include <QTextStream>
#include <QImage>
#include <QDir>
#include <QFileInfo>
#include <QInputDialog>
#include <QPainter>
#include <QSettings>
#include "qtColormaps.h"
void delay(int seconds)
{
    QTime waitTime = QTime::currentTime().addSecs(seconds);
    while (QTime::currentTime() < waitTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

HandEyeCalibrateDialog::HandEyeCalibrateDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HandEyeCalibrateDialog),
    mHiWinRobot(new HiWinRobot(this)),
    mStopContinueMove(true)
{
    ui->setupUi(this);

    mCamera = mRoot[itmCameras][itmBySerialNo][0];

    ui->portLineEdit->setValidator(new QIntValidator(1000,9999));

    QSettings settings;

    ui->portLineEdit->setText(settings.value("HiWinRobot/ServerPort", 4000).toString());


    mDepthColors     = colormap("jet",  256 - FixedColors);
    connect(mHiWinRobot, SIGNAL(client_data_received(QString,int)), this, SLOT(on_client_data_received(QString,int)));
    connect(mHiWinRobot, SIGNAL(server_data_sended(QString)), this, SLOT(on_server_data_sended(QString)));
    connect(&mGrabImageTimer, SIGNAL(timeout()), this, SLOT(on_grab_Image()));
    ui->leftImage->setScaledContents(true);
    ui->leftImage->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->rightImage->setScaledContents(true);
    ui->rightImage->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    nxLibInitialize(true);
    loadPoses();
    connect(mReadParametersTimer, SIGNAL(timeout()), this, SLOT(on_readParameters()));
    connect(ui->exposureSlider,           SIGNAL(valueChanged(int)), this, SLOT(on_parameter_changed()));
    mReadParametersTimer->start(1000);
    on_readParameters(true);

}

HandEyeCalibrateDialog::~HandEyeCalibrateDialog()
{
    nxLibCloseTcpPort();
    nxLibFinalize();
    delete ui;
}

void HandEyeCalibrateDialog::offline()
{
    ui->SocketButton->setText(tr("ON LINE"));
    mHiWinRobot->stop();
    ui->CalibratePushButton->setEnabled(false);
    mOnLine = false;
}

void HandEyeCalibrateDialog::onLine()
{
    ui->SocketButton->setText(tr("OFF LINE"));
    mHiWinRobot->start();
    ui->CalibratePushButton->setEnabled(true);
    mOnLine = true;

}

void HandEyeCalibrateDialog::parseRobotPose(QString message, double &x, double &y, double &z, double &a, double &b, double &c)
{

    QRegularExpression re("(-*[\\d.]+)");
    QRegularExpressionMatchIterator head, iter = re.globalMatch(message);
    head = iter;
    int count = 0;
    while (iter.hasNext())
    {
        ++count;
        iter.next();
    }
    if (count != 6) return;
    QRegularExpressionMatch match;
    qDebug() << "Robot Pose: " << message;
    match = head.next();
    x = match.captured(1).toDouble();
    qDebug() << "x = " <<x;
    match = head.next();
    y = match.captured(1).toDouble();
    qDebug() << "y = " << y;
    match = head.next();
    z = match.captured(1).toDouble();
    qDebug() << "z = " << z;
    match = head.next();
    a = match.captured(1).toDouble();
    qDebug() << "a = " << a;
    match = head.next();
    b = match.captured(1).toDouble();
    qDebug() << "b = " << b;
    match = head.next();
    c = match.captured(1).toDouble();
    qDebug() << "c = " << c;
}

void HandEyeCalibrateDialog::loadPoses()
{

        QString path = qApp->applicationDirPath() + QString("\\poses");
        QDir dir(path);
        dir.setNameFilters(QStringList() << "*.dat");
        dir.setFilter(QDir::Files);
        ui->posesFileComboBox->clear();
        foreach (QFileInfo info, dir.entryInfoList()) {
            ui->posesFileComboBox->addItem(info.fileName());
        }
        QString tmp = mSettings.value("RobotPoses").toString();
        int index = ui->posesFileComboBox->findText(tmp);
        if (index != -1)
        {
            ui->posesFileComboBox->setCurrentIndex(index);
        }
}

QPixmap HandEyeCalibrateDialog::convertXYZBuffer(const std::vector<float> &depthBuffer, int bufferWidth, int bufferHeight)
{
    if (depthBuffer.empty()) return QPixmap();

        // Convert depth map to 8bit with indexed colors
        std::vector<unsigned char> buf;
        buf.resize(depthBuffer.size() / 3);
        for (size_t i = 0; i < buf.size(); i++) {
            float z = depthBuffer[3*i + 2]; // We only use the Z-Coordinate for the 2D display
            if (z != z) { // Skip NaN values
                buf[i] = 255;
                continue;
            }
            buf[i] = valueToColorIndex(z, zMinValue, zColorRepetitionDistance);
        }
        QImage image;
        image = QImage(&buf[0], bufferWidth, bufferHeight, bufferWidth, QImage::Format_Indexed8);
        image.setColorTable(mDepthColors);

        QPixmap map = imageToPixmap(image);
        return map;
}


int mapRangeForwardBackward(int value, int maxValue) {
    int wraps  = value / maxValue;
    int remain = value % maxValue;
    if (wraps % 2 == 0) { // Invert every second color range to have no color space wraps
        return remain;
    } else {
        return maxValue - remain;
    }
}
inline unsigned char HandEyeCalibrateDialog::valueToColorIndex(float z, float offset, float colorRepetitionDistance) {
    float scale = (float) (255 - FixedColors) / colorRepetitionDistance;
    int val = (int)floor(0.5 + std::fabs((z - offset - (!mColorsSymmetric ? 0 : colorRepetitionDistance / 2)) * scale));
    return mapRangeForwardBackward(val, 255-FixedColors);
}

QPixmap HandEyeCalibrateDialog::imageToPixmap(QImage &image, int width, int height, int x, int y)
{
    try
    {
        int h = height >= 0 ? height : image.height();
        int w = width  >= 0 ? width  : image.width ();
        QPixmap map(w, h);
        QPainter painter(&map);
        map.fill();
        painter.drawImage(x, y, image);
        return map;

    }
    catch(QException &e)
    {
        return QPixmap();
    }
}

void HandEyeCalibrateDialog::updateLabelAndSlider(bool forceSliderPosition, QLabel *label, QSlider *slider, double inputValue, double scale, double specialValue, const std::string &specialText, double factor)
{

    label->setEnabled(true);
    slider->setEnabled(true);

    double value = inputValue * factor;

    if (!label || !slider) return;

    if (forceSliderPosition && !slider->isSliderDown()) {
        slider->setValue((int)floor(value/scale + 0.5));
    }

    bool valueDiverges = std::fabs(value / scale - slider->value()) > 0.5;

    if (value == specialValue) {
        label->setText(specialText.c_str());
    } else if (!valueDiverges || forceSliderPosition) {
        label->setNum(slider->value()*scale);
    } else {
        label->setNum((int)floor(value/scale + 0.5)*scale);
    }

    if (valueDiverges && !forceSliderPosition) {
        QPalette palette = label->palette();
        palette.setBrush(QPalette::Active,   QPalette::WindowText, QBrush(QColor(255, 0, 0))); // Mark invalid slider positions in red+bold
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, QBrush(QColor(255, 0, 0))); // Mark invalid slider positions in red+bold
        label->setPalette(palette);
        QFont font = label->font();
        font.setBold(true);
        label->setFont(font);
    } else {
        QPalette palette = label->palette();
        palette.setBrush(QPalette::Active,   QPalette::WindowText, QBrush(QColor(0, 0, 0)));
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, QBrush(QColor(0, 0, 0)));
        label->setPalette(palette);
        QFont font = label->font();
        font.setBold(false);
        label->setFont(font);
    }
}

void HandEyeCalibrateDialog::on_SocketButton_clicked()
{
    QSettings settings;
    settings.setValue("HiWinRobot/ServerPort", ui->portLineEdit->text().toUInt());
    if (mOnLine)
    {
       offline();
    }
    else
    {
        onLine();
    }
}

void HandEyeCalibrateDialog::on_readParameters(bool forceSliderPositions)
{
    if (mPendingChanges) return;
    bool f = forceSliderPositions;

    NxLibItem root; // Reference to the API tree root

    // Create an object referencing the camera's tree item, for easier access:
    NxLibItem captureParams = root[itmCameras][itmBySerialNo][0][itmParameters][itmCapture];
        captureParams[itmAutoExposure] = true;
    ui->autoExposureChk->setChecked(captureParams[itmAutoExposure].asBool());
    updateLabelAndSlider(f, ui->exposureLabel, ui->exposureSlider, captureParams[itmExposure].asDouble(), 0.01);

    mPendingChanges = false;
}

void HandEyeCalibrateDialog::on_MoveButton_clicked()
{
    if (!mOnLine) return;
    mCalibrating = false;
    mHiWinRobot->move(ui->XLineEdit->text().toDouble(),
                      ui->YLineEdit->text().toDouble(),
                      ui->ZLineEdit->text().toDouble(),
                      ui->ALineEdit->text().toDouble(),
                      ui->BLineEdit->text().toDouble(),
                      ui->CLineEdit->text().toDouble());
}

void HandEyeCalibrateDialog::on_GetPoseButton_clicked()
{
   if (!mOnLine) return;
   mHiWinRobot->getPose();
}


void HandEyeCalibrateDialog::on_client_data_received(QString message, int)
{

    ui->logListWidget->addItem(QString("R: %1").arg(message));
    ui->logListWidget->setCurrentRow(ui->logListWidget->count() - 1);
    QRegularExpression re("(-*[\\d.]+)");
    QRegularExpressionMatchIterator head, iter = re.globalMatch(message);
    head = iter;
    int count = 0;
    while (iter.hasNext())
    {
        ++count;
        iter.next();
    }
    if (count == 0) return;
    QRegularExpressionMatch match;
    double condition = 0;
    switch (count) {
    case 1:
        match = head.next();
        bool isNumber;
        condition = match.captured(1).toDouble(&isNumber);
        if (isNumber && condition == 11)
        {
            qDebug() << "ACK(11) or DONE(12): " << "ACK(11)";
        }
        else if (isNumber && condition == 12)
        {
            mHiWinRobot->setMoveDone(true);
            qDebug() << "ACK(11) or DONE(12): " << "DONE(12)";
        }
        else
        {
            qDebug() << "ACK(11) or DONE(12): " <<  match.captured(1);
        }
        break;
    case 6:
        qDebug() << "Robot Pose: ";
        RobotPoseStruct pose;
        match = head.next();
        qDebug() << "X: " << match.captured(1);
        pose.x = match.captured(1).toDouble();
        match = head.next();
        qDebug() << "Y: " << match.captured(1);
        pose.y = match.captured(1).toDouble();
        match = head.next();
        qDebug() << "Z: " << match.captured(1);
        pose.z = match.captured(1).toDouble();
        match = head.next();
        qDebug() << "A: " << match.captured(1);
        pose.a = match.captured(1).toDouble();
        match = head.next();
        qDebug() << "B: " << match.captured(1);
        pose.b = match.captured(1).toDouble();
        match = head.next();
        qDebug() << "C: " << match.captured(1);
        pose.c = match.captured(1).toDouble();
        mPoseQueue.enqueue(pose);
        if (ui->addToRobtoPoseCheckBox->isChecked())
        {
            ui->robotPoseListWidget->addItem(message);
        }
        break;
    }

}

void HandEyeCalibrateDialog::on_server_data_sended(QString message)
{
    qDebug() << QString("S: %1").arg(message);
    ui->logListWidget->addItem(QString("S: %1").arg(message));
    ui->logListWidget->setCurrentRow(ui->logListWidget->count() - 1);
}


void HandEyeCalibrateDialog::on_robotPoseListWidget_itemDoubleClicked(QListWidgetItem *item)
{
   if (item == nullptr) return;
   double x, y, z, a, b, c; x = y = z = a = b = c = 0;
   parseRobotPose(item->data(Qt::DisplayRole).toString(), x, y, z, a, b, c);
   qDebug() << QString("x=%1,y=%2,z=%3,a=%4,b=%5,c=%6").arg(x).arg(y).arg(z).arg(a).arg(b).arg(c);
   ui->XLineEdit->setText(QString::number(x));
   ui->YLineEdit->setText(QString::number(y));
   ui->ZLineEdit->setText(QString::number(z));
   ui->ALineEdit->setText(QString::number(a));
   ui->BLineEdit->setText(QString::number(b));
   ui->CLineEdit->setText(QString::number(c));

}

void HandEyeCalibrateDialog::on_poseUpPushButton_clicked()
{
   if (ui->robotPoseListWidget->selectedItems().count() == 0) return;
   QListWidgetItem *current = ui->robotPoseListWidget->currentItem();
   int index = ui->robotPoseListWidget->row(current);
   if (index == 0) return;
   QListWidgetItem *tmp =  ui->robotPoseListWidget->takeItem(index - 1);
   ui->robotPoseListWidget->insertItem(index - 1, current);
   ui->robotPoseListWidget->insertItem(index, tmp);

}

void HandEyeCalibrateDialog::on_poseDownPushButton_clicked()
{

   if (ui->robotPoseListWidget->selectedItems().count() == 0) return;
   QListWidgetItem *current = ui->robotPoseListWidget->currentItem();
   int index = ui->robotPoseListWidget->row(current);
   if (index == ui->robotPoseListWidget->count() - 1) return;
   QListWidgetItem *tmp =  ui->robotPoseListWidget->takeItem(index + 1);
   ui->robotPoseListWidget->insertItem(index + 1, current);
   ui->robotPoseListWidget->insertItem(index, tmp);

}

void HandEyeCalibrateDialog::on_removePosePushButton_clicked()
{
   if (ui->robotPoseListWidget->selectedItems().count() == 0) return;
   int index = ui->robotPoseListWidget->currentRow();
   QListWidgetItem *item = ui->robotPoseListWidget->takeItem(index);
   delete item;
}

void HandEyeCalibrateDialog::on_clearPosePushButton_clicked()
{
   ui->robotPoseListWidget->clear();
}

void HandEyeCalibrateDialog::on_MovePushButton_clicked()
{
   if (ui->robotPoseListWidget->selectedItems().count() == 0) return;
   QListWidgetItem *item = ui->robotPoseListWidget->currentItem();
   double x, y, z, a, b, c; x = y = z = a = b = c = 0;
   parseRobotPose(item->data(Qt::DisplayRole).toString(), x, y, z, a, b, c);
   ui->XLineEdit->setText(QString::number(x));
   ui->YLineEdit->setText(QString::number(y));
   ui->ZLineEdit->setText(QString::number(z));
   ui->ALineEdit->setText(QString::number(a));
   ui->BLineEdit->setText(QString::number(b));
   ui->CLineEdit->setText(QString::number(c));
   if (!mOnLine) return;
   mHiWinRobot->move(ui->XLineEdit->text().toDouble(),
                     ui->YLineEdit->text().toDouble(),
                     ui->ZLineEdit->text().toDouble(),
                     ui->ALineEdit->text().toDouble(),
                     ui->BLineEdit->text().toDouble(),
                     ui->CLineEdit->text().toDouble());
    ui->logListWidget->addItem(QString("S: [1,0,%1,%2,%3,%4,%5,%6]").arg(ui->XLineEdit->text())
                               .arg(ui->YLineEdit->text())
                               .arg(ui->ZLineEdit->text())
                               .arg(ui->ALineEdit->text())
                               .arg(ui->BLineEdit->text())
                               .arg(ui->CLineEdit->text()));
    int retryCount = 0;
    while (mHiWinRobot->moveDone() == false && retryCount < 30)
    {
        qDebug() << "Moving ...";
        delay(1);
        retryCount++;
    }

}

void HandEyeCalibrateDialog::on_loadRobotPosePushButton_clicked()
{
    if (ui->posesFileComboBox->currentIndex() == -1) return;

    QFile file(qApp->applicationDirPath() + "\\poses\\" + ui->posesFileComboBox->currentText());
    if (!file.open(QIODevice::ReadOnly)) return;

    QDataStream stream(&file);
    stream.setVersion(QDataStream::Qt_5_9);

    QList<QString> list;

    stream >> list;
    file.close();

    ui->robotPoseListWidget->clear();
    for (int i = 0; i < list.count(); i++)
    {
        ui->robotPoseListWidget->addItem(list[i]);
    }



}

void HandEyeCalibrateDialog::on_saveRobotPosePushButton_clicked()
{
    if (ui->robotPoseListWidget->count() == 0) return;

  //  QString fileName = QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
    QString fileName = ui->posesFileComboBox->currentText().section(".",0,0);

    bool ok;
    QString text = QInputDialog::getText(this, tr("Robot Poses File"),
                                         tr("File name:"), QLineEdit::Normal,
                                         fileName, &ok);
    if (ok && !text.isEmpty())
    {
        mSettings.setValue("RobotPoses", text + ".dat");
        QFile file(qApp->applicationDirPath() + "\\poses\\" + text + ".dat");
        if (!file.open(QIODevice::WriteOnly)) return;

        QDataStream stream(&file);
        stream.setVersion(QDataStream::Qt_5_9);

        QList<QString> list;
        for (int i = 0; i < ui->robotPoseListWidget->count(); i++)
        {
            list <<  ui->robotPoseListWidget->item(i)->data(Qt::DisplayRole).toString();
        }

        stream << list;

        file.close();

        loadPoses();

    }

}

void HandEyeCalibrateDialog::on_logListWidget_itemDoubleClicked(QListWidgetItem *)
{
    ui->logListWidget->clear();
}

void HandEyeCalibrateDialog::on_CalibratePushButton_clicked()
{
    ui->grabImageCheckBox->setChecked(false);
    ui->addToRobtoPoseCheckBox->setChecked(false);
    on_grabImageCheckBox_clicked(false);
    mCalibrating = true;
    mPoseQueue.clear();
    ui->calibrateLogListWidget->clear();
   if (ui->robotPoseListWidget->count() == 0) return;

    std::vector<unsigned char> buffer;


    qDebug() << "This is a demo for doing robot Hand-Eye Calibration with NxLib";
    try
    {
        QString path = qApp->applicationDirPath() + QString("\\images");
        QDir dir(path);
        dir.setNameFilters(QStringList() << "*.tiff");
        dir.setFilter(QDir::Files);
        foreach (QString dirFile, dir.entryList()) {
            dir.remove(dirFile);
        }


       ui->calibrateLogListWidget->addItem("Opening NxLib and waiting for cameras to be detected");
       ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
       qDebug() <<  "Opening NxLib and waiting for cameras to be detected";


        // Create an object referencing the camera's tree item, for easier access:
        if (!mCamera.exists() || (mCamera[itmType] != valStereo)) {
            ui->calibrateLogListWidget->addItem("Please connect a single stereo camera to your computer");
            ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
           qDebug()<< "Please connect a single stereo camera to your computer";
           return;
        }

        std::string serial = mCamera[itmSerialNumber].asString();
        ui->calibrateLogListWidget->addItem(QString("Opening camera %1").arg(serial.c_str()));
        ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
        qDebug() << (QString("Opening camera %1").arg(serial.c_str()));
        NxLibCommand open(cmdOpen); // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
        open.parameters()[itmCameras] = serial; // Set parameters for the open command
        open.execute();

        int port;
        nxLibOpenTcpPort(9998, &port);


        NxLibCommand discardPatterns(cmdDiscardPatterns);
        discardPatterns.execute();
        ui->calibrateLogListWidget->addItem("Discard Patterns");
        ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));

        NxLibItem captureParams = mCamera[itmParameters][itmCapture];
        //captureParams[itmFrontLight] = true;
        captureParams[itmProjector] = false;
        captureParams[itmAutoExposure] = true;
        captureParams[itmAutoGain] = true;

        bool patternDecoded = false;
        QList<std::string> rawPoses;
        QList<std::string> robotPoses;

        for (int i= 0; i < ui->robotPoseListWidget->count(); i++)
        {
            if (mCalibrating == false)
                break;
            ui->calibrateLogListWidget->addItem(QString("---%1---").arg(i));

             QListWidgetItem *item = ui->robotPoseListWidget->item(i);
             double x, y, z, a, b, c; x = y = z = a = b = c = 0;
             parseRobotPose(item->data(Qt::DisplayRole).toString(), x, y, z, a, b, c);
             if (!mOnLine) return;
             ui->calibrateLogListWidget->addItem(QString("Moving To [%1,%2,%3,%4,%5,%6]").arg(x)
                                                 .arg(y)
                                                 .arg(z)
                                                 .arg(a)
                                                 .arg(b)
                                                 .arg(c));
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
             mHiWinRobot->move(x,y,z,a,b,c);
             int retryCount = 0;
             while (mHiWinRobot->moveDone() == false && retryCount < 30)
             {
                 qDebug() << "Moving ...";
                 delay(1);
                 retryCount++;
             }
             ui->calibrateLogListWidget->addItem("Move Done.");
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));

             NxLibCommand capture(cmdCapture);
             ui->calibrateLogListWidget->addItem("Capture.");
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
             capture.execute();

             int returnCode, width, height;
             std::vector<uchar> rBinaryData, lBinaryData;
             std::vector<uchar> rBinaryDataO, lBinaryDataO;

             //Left Image
             NxLibItem leftImg  = mCamera[itmImages][itmRaw][itmLeft ];
             leftImg.getBinaryDataInfo(&width, &height, 0, 0,0,0);
             leftImg.getBinaryData(&returnCode, lBinaryData, 0);
             QImage lImage(&lBinaryData[0], width, height, QImage::Format_Grayscale8);
             ui->leftImage->setPixmap(QPixmap::fromImage(lImage, Qt::AutoColor));

             //Right Image
             NxLibItem rightImg = mCamera[itmImages][itmRaw][itmRight];
             rightImg.getBinaryDataInfo(&width, &height, 0,0,0,0);
             rightImg.getBinaryData(&returnCode, rBinaryData, 0);
             QImage rImage(&rBinaryData[0], width, height, QImage::Format_Grayscale8);
             ui->rightImage->setPixmap(QPixmap::fromImage(rImage, Qt::AutoColor));

             NxLibCommand collect(cmdCollectPattern);
             try
             {
                collect.parameters()[itmBuffer] = false;
                 collect.execute();
                 ui->calibrateLogListWidget->addItem("Collect Pattern, itmBuffer = false.");
                 ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));

             }
             catch(NxLibException &e)
             {
                 ui->calibrateLogListWidget->addItem(QString("Collect Photo1 Pattern: NxLibException, ErrorCode %1, ErrorText: %2").arg(e.getErrorCode())
                                                     .arg(QString::fromStdString(e.getErrorText())));

                 ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
                 if (e.getErrorCode() == NxLibExecutionFailed)
                 {
                     if (collect.result()[itmErrorSymbol].asString() == errPatternNotFound) continue;
                     throw;
                 }
             }



             delay(2);

             try
             {
                 if (!patternDecoded) collect.parameters()[itmDecodeData] = true;
                 collect.execute();
                 ui->calibrateLogListWidget->addItem("Collect Pattern, itmDecodeData = true.");
                 ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));

             }
             catch(NxLibException &e)
             {
                 ui->calibrateLogListWidget->addItem(QString("Collect Photo2 Pattern: NxLibException, ErrorCode %1, ErrorText: %2").arg(e.getErrorCode())
                                                     .arg(QString::fromStdString(e.getErrorText())));
                 ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
                 if (e.getErrorCode() == NxLibExecutionFailed)
                 {
                     if (collect.result()[itmErrorSymbol].asString() == errPatternNotFound    ) continue;
                     if (collect.result()[itmErrorSymbol].asString() == errPatternNotDecodable) continue;
                     throw;
                 }
             }

             if (!patternDecoded)
             {
                 double spacing = collect.result()[itmGridSpacing].asDouble();
                 ui->calibrateLogListWidget->addItem(QString("Detected pattern with grid spacing %1 mm.").arg(spacing));
                 ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
                 qDebug() << "Detected pattern with grid spacing " << spacing << "mm.";
                 mRoot[itmParameters][itmPattern][itmGridSpacing].set(spacing);
                 patternDecoded = true;
             }


             //Left Image
             NxLibItem leftImgO  = mCamera[itmImages][itmWithOverlay][itmLeft ];
             leftImgO.getBinaryDataInfo(&width, &height, 0, 0,0,0);
             leftImgO.getBinaryData(&returnCode, lBinaryDataO, 0);

             QImage lImageO(&lBinaryDataO[0], width, height, QImage::Format_RGB888);
             ui->leftImage->setPixmap(QPixmap::fromImage(lImageO, Qt::AutoColor));

             //Right Image
             NxLibItem rightImgO = mCamera[itmImages][itmWithOverlay][itmRight];
             rightImgO.getBinaryDataInfo(&width, &height, 0,0,0,0);
             rightImgO.getBinaryData(&returnCode, rBinaryDataO, 0);

             QImage rImageO(&rBinaryDataO[0], width, height, QImage::Format_RGB888);
             ui->rightImage->setPixmap(QPixmap::fromImage(rImageO, Qt::AutoColor));

             ui->calibrateLogListWidget->addItem("GetPose.");
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
             qDebug() << "GetPose";
             mHiWinRobot->getPose();
             retryCount = 0;
             while (mPoseQueue.isEmpty() && retryCount < 30)
             {
                 delay(1);
                 retryCount++;
             }
             RobotPoseStruct pose = mPoseQueue.dequeue();

             QString temp = QString("Current Pose: [%1,%2,%3,%4,%5,%6]").arg(pose.x)
                     .arg(pose.y)
                     .arg(pose.z)
                     .arg(pose.a)
                     .arg(pose.b)
                     .arg(pose.c);
             ui->calibrateLogListWidget->addItem(temp);
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));

             qDebug() << temp;

             int patternCount = mRoot[itmParameters][itmPatternCount].asInt();

             if (robotPoses.count() + 1 == patternCount)
             {
                 rawPoses.append(temp.toStdString());
                std::string json = NxLibTransformationHelper::EulerAnglesToTransformationJSON(pose.x, pose.y, pose.z, pose.a, pose.b, pose.c);
                robotPoses.append(json);

                temp =QString("Pose Count = %1, Pattern Count = %2").arg(robotPoses.count()).arg(patternCount);
                ui->calibrateLogListWidget->addItem(temp);
                ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));

                temp = QString::fromStdString(json);
                ui->calibrateLogListWidget->addItem(temp);
                ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
                qDebug() << temp;

                //Saving Left Image
                 if (ui->saveImageCheckBox->isChecked())
                 {
                     lImage.save(qApp->applicationDirPath() + QString("\\images\\%1-L.tiff").arg(i), "tiff");
                 }

                 }

                //Saving RightImage
                 if (ui->saveImageCheckBox->isChecked())
                 {
                     rImage.save(qApp->applicationDirPath() + QString("\\images\\%1-R.tiff").arg(i), "tiff");
                 }

        }

        if (mCalibrating)
        {
            ui->calibrateLogListWidget->addItem("------");


             double x, y, z, a, b, c; x = y = z = a = b = c = 0;
             x = mSettings.value("Origin/X",0).toDouble();
             y = mSettings.value("Origin/Y",0).toDouble();
             z = mSettings.value("Origin/Z",0).toDouble();
             a = mSettings.value("Origin/A",0).toDouble();
             b = mSettings.value("Origin/B",0).toDouble();
             c = mSettings.value("Origin/C",0).toDouble();
             ui->calibrateLogListWidget->addItem(QString("Moving To [%1,%2,%3,%4,%5,%6]").arg(x)
                                                 .arg(y)
                                                 .arg(z)
                                                 .arg(a)
                                                 .arg(b)
                                                 .arg(c));
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
             mHiWinRobot->move(x,y,z,a,b,c);
             while (mHiWinRobot->moveDone() == false)
             {
                 delay(1);
             }
             ui->calibrateLogListWidget->addItem("Move Done.");
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));


             NxLibCommand calibrate(cmdCalibrateHandEye);
             QString tmp;
             for (int i = 0; i < robotPoses.count(); i++)
             {

                calibrate.parameters()[itmTransformations][i].setJson(robotPoses[i], false);
                tmp = QString::fromStdString(rawPoses[i]);
                ui->calibrateLogListWidget->addItem(QString("Calibrate Robot Pose[%1] = %2").arg(i).arg(tmp));
                tmp = QString::fromStdString(robotPoses[i]);
                ui->calibrateLogListWidget->addItem(QString("Calibrate Transformations[%1] = %2").arg(i).arg(tmp));
                ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
             }
             calibrate.parameters()[itmSetup].set(ui->movingCameraRadioButton->isChecked() ? valMoving : valFixed);
             // You might initialize the pattern pose and/or camera link here to speed up convergence
             //calibrate.Parameters()[NxLib.itmPatternPose].SetJson(/*insert a valid json transformation here*/,false);
             //calibrate.Parameters()[NxLib.itmLink       ].SetJson(/*insert a valid json transformation here*/,false);

             ui->calibrateLogListWidget->addItem("Calibrating.");
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
             // Run calibration
             calibrate.execute(false);
             int retryCount = 0;
             while (!calibrate.finished() && retryCount < 30)
             {
                 delay(1);
                 ++retryCount;
             }
             ui->calibrateLogListWidget->addItem("Calibrate Finish.");
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
             QString calibrateResult = QString::fromStdString(calibrate.result().asJson(true, 4, false));
             QString calibrateLink = QString::fromStdString(mCamera[itmLink].asJson(true, 4, false));

             qDebug() << "Result:" << calibrateResult;
             qDebug() << "Link:" << calibrateLink;
             ui->calibrateLogListWidget->addItem("Calibrate Result: " + calibrateResult);
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
             ui->calibrateLogListWidget->addItem("Calibrate Link: " + calibrateLink);
             ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));

             if (calibrate.successful())
             {
                 ui->calibrateLogListWidget->addItem("Calibration successful!");
                 ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
                 qDebug() << "Calibration successful!";
             }
             if (ui->notEEPROM->isChecked())
             {
                 ui->calibrateLogListWidget->addItem("Discarding link");
                 ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
                 qDebug() << "Discarding link";

             }
             else
             {
                 ui->calibrateLogListWidget->addItem("Storing link in eeprom...");
                 ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
                 qDebug() << "Storing link in eeprom...";
                 try
                 {
                    NxLibCommand store(cmdStoreCalibration);
                    store.parameters()[itmLink] = true;
                    store.execute();
                 }
                 catch(NxLibException &e)
                 {
                     ui->calibrateLogListWidget->addItem(QString("Store Calibration: NxLibException, ErrorCode %1, ErrorText: %2, ItemPath: %3").arg(e.getErrorCode())
                                                         .arg(QString::fromStdString(e.getErrorText()))
                                                         .arg(QString::fromStdString(e.getItemPath())));
                     ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
                 }
             }


        }

    }
    catch (NxLibException &e)
    {
       ui->calibrateLogListWidget->addItem(QString("NxLibException, ErrorCode %1, ErrorText: %2").arg(e.getErrorCode())
                                           .arg(QString::fromStdString(e.getErrorText())));
        qDebug() << QString("An NxLib API error with code %1 (%2) occurred while accessing item %3.")
                    .arg(e.getErrorCode())
                    .arg(e.getErrorText().c_str())
                    .arg(e.getItemPath().c_str());
        if (e.getErrorCode() == NxLibExecutionFailed) qDebug() << QString("/Execute: %1").arg(NxLibItem(itmExecute).asJson(true).c_str());


    }
    catch (...)
    {
        ui->calibrateLogListWidget->addItem("Something, somewhere went terribly wrong!");
        qDebug() << "Something, somewhere went terribly wrong!";

    }
    try
    {
        ui->calibrateLogListWidget->addItem("Closing camera");
        ui->calibrateLogListWidget->setCurrentRow((ui->calibrateLogListWidget->count() - 1));
        qDebug() << "Closing camera";
        NxLibCommand (cmdClose).execute();
    }
    catch(...)
    {

    }

    mCalibrating = false;
    ui->saveLogPushButton->click();

}

void HandEyeCalibrateDialog::on_groupBox_3_clicked()
{

}

void HandEyeCalibrateDialog::on_calibrateCheckBox_clicked()
{

}

void HandEyeCalibrateDialog::on_pushButton_clicked()
{

    if (ui->calibrateLogListWidget->count() == 0) return;

    QFile file(qApp->applicationDirPath() + "\\CalibrateLog.txt");
    if (!file.open(QIODevice::WriteOnly)) return;

    QTextStream stream(&file);

    for (int i = 0; i < ui->calibrateLogListWidget->count(); i++)
    {
        stream << ui->calibrateLogListWidget->item(i)->data(Qt::DisplayRole).toString() << "\n";
    }


    file.close();
}

void HandEyeCalibrateDialog::on_pushButton_2_clicked()
{
   //
    nxLibInitialize(true);

    NxLibItem root; // Reference to the API tree root

    // Create an object referencing the camera's tree item, for easier access:
    NxLibItem camera = root[itmCameras][itmBySerialNo][0];
    if (!camera.exists() || (camera[itmType] != valStereo)) {
       qDebug()<< "Please connect a single stereo camera to your computer";
    }

    std::string serial = camera[itmSerialNumber].asString();
    NxLibCommand open(cmdOpen); // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
    open.parameters()[itmCameras] = serial; // Set parameters for the open command
    open.execute();

    NxLibItem captureParams = camera[itmParameters][itmCapture];
    //captureParams[itmFrontLight] = true;
    captureParams[itmProjector] = true;
    captureParams[itmAutoExposure] = true;
    captureParams[itmAutoGain] = true;

    NxLibCommand capture(cmdCapture);
    capture.execute();

//    int returnCode, width, height;
//    std::vector<uchar> binaryData;

//    NxLibItem leftImg  = camera[itmImages][itmRaw][itmLeft ];
//    leftImg.getBinaryDataInfo(&width, &height, 0,0,0,0);
//    leftImg.getBinaryData(&returnCode, binaryData, 0);

//    QImage image(&binaryData[0], width, height, QImage::Format_Grayscale8);

//    ui->leftImage->setScaledContents(true);
//    ui->leftImage->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
//    ui->leftImage->setPixmap(QPixmap::fromImage(image, Qt::AutoColor));

//    NxLibItem rightImg = camera[itmImages][itmRaw][itmRight];
//    rightImg.getBinaryDataInfo(&width, &height, 0,0,0,0);
//    rightImg.getBinaryData(&returnCode, binaryData, 0);

//    QImage rImage(&binaryData[0], width, height, QImage::Format_Grayscale8);

//    ui->rightImage->setScaledContents(true);
//    ui->rightImage->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
//    ui->rightImage->setPixmap(QPixmap::fromImage(rImage, Qt::AutoColor));

    int returnCode, width, height;
    std::vector<uchar> binaryData;

    NxLibItem leftImg  = camera[itmImages][itmRaw][itmLeft ];
    leftImg.getBinaryDataInfo(&width, &height, 0,0,0,0);
    leftImg.getBinaryData(&returnCode, binaryData, 0);

    QImage lImage(&binaryData[0], width, height, QImage::Format_Grayscale8);

    ui->leftImage->setScaledContents(true);
    ui->leftImage->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->leftImage->setPixmap(QPixmap::fromImage(lImage, Qt::AutoColor));

    NxLibItem rightImg = camera[itmImages][itmRaw][itmRight];
    rightImg.getBinaryDataInfo(&width, &height, 0,0,0,0);
    rightImg.getBinaryData(&returnCode, binaryData, 0);

    QImage rImage(&binaryData[0], width, height, QImage::Format_Grayscale8);

    ui->rightImage->setScaledContents(true);
    ui->rightImage->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->rightImage->setPixmap(QPixmap::fromImage(rImage, Qt::AutoColor));


    NxLibCommand (cmdClose).execute();
    nxLibFinalize();
}

void HandEyeCalibrateDialog::on_continueMovePushButton_clicked()
{

    if (mStopContinueMove == false)
    {
        return;
    }
    mStopContinueMove = false;
    try
    {
        if (ui->robotPoseListWidget->count() == 0) return;
        for (int i = 0; i < ui->robotPoseListWidget->count(); i++)
        {
            if (mStopContinueMove)
            {
                break;
            }
            QListWidgetItem *item = ui->robotPoseListWidget->item(i);
            double x, y, z, a, b, c; x = y = z = a = b = c = 0;
            parseRobotPose(item->data(Qt::DisplayRole).toString(), x, y, z, a, b, c);
            ui->XLineEdit->setText(QString::number(x));
            ui->YLineEdit->setText(QString::number(y));
            ui->ZLineEdit->setText(QString::number(z));
            ui->ALineEdit->setText(QString::number(a));
            ui->BLineEdit->setText(QString::number(b));
            ui->CLineEdit->setText(QString::number(c));
            if (!mOnLine) return;
            mHiWinRobot->move(ui->XLineEdit->text().toDouble(),
                              ui->YLineEdit->text().toDouble(),
                              ui->ZLineEdit->text().toDouble(),
                              ui->ALineEdit->text().toDouble(),
                              ui->BLineEdit->text().toDouble(),
                              ui->CLineEdit->text().toDouble());
             ui->logListWidget->addItem(QString("S: [1,0,%1,%2,%3,%4,%5,%6]").arg(ui->XLineEdit->text())
                                        .arg(ui->YLineEdit->text())
                                        .arg(ui->ZLineEdit->text())
                                        .arg(ui->ALineEdit->text())
                                        .arg(ui->BLineEdit->text())
                                        .arg(ui->CLineEdit->text()));
            int retryCount = 0;
            while (mHiWinRobot->moveDone() == false && retryCount < 30)
            {
                qDebug() << "Moving ...";
                delay(1);
                retryCount++;
            }
            ui->logListWidget->addItem("Move Done.");

        }

    }
    catch(...)
    {
    }
    mStopContinueMove = true;
}

void HandEyeCalibrateDialog::on_stopPushButton_clicked()
{
   mStopContinueMove = true;
}

void HandEyeCalibrateDialog::on_stopCalibratePushButton_clicked()
{
    mCalibrating = false;
}

void HandEyeCalibrateDialog::on_saveLogPushButton_clicked()
{
    if (ui->calibrateLogListWidget->count() == 0) return;

    QFile file(qApp->applicationDirPath() + "\\CalibrateLog.txt");
    if (!file.open(QIODevice::WriteOnly)) return;

    QTextStream stream(&file);

    for (int i = 0; i < ui->calibrateLogListWidget->count(); i++)
    {
        stream << ui->calibrateLogListWidget->item(i)->data(Qt::DisplayRole).toString() << "\n";
    }


    file.close();
}


void HandEyeCalibrateDialog::on_grabImageCheckBox_clicked(bool checked)
{
    if (checked)
    {
        // Create an object referencing the camera's tree item, for easier access:
        if (!mCamera.exists() || (mCamera[itmType] != valStereo)) {
           return;
        }

        std::string serial = mCamera[itmSerialNumber].asString();
        NxLibCommand open(cmdOpen); // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
        open.parameters()[itmCameras] = serial; // Set parameters for the open command
        open.execute();

        //NxLibCommand convertTransformation(cmdConvertTransformation);
        //convertTransformation.parameters()[itmTransformation] = mCamera[itmLink].asJson();
        //convertTransformation.execute();


        mGrabImageTimer.start(1000);
        on_grab_Image();
    }
    else
    {
        mGrabImageTimer.stop();

        try
        {
            NxLibCommand (cmdClose).execute();
        }
        catch(...)
        {

        }
    }
}

void HandEyeCalibrateDialog::on_grab_Image()
{
    try
    {
        NxLibItem captureParams = mCamera[itmParameters][itmCapture];
        //captureParams[itmFrontLight] = true;
        captureParams[itmProjector] = false;
        captureParams[itmAutoExposure] = true;
        captureParams[itmAutoGain] = true;

        NxLibCommand capture(cmdCapture);
        capture.execute();

        int returnCode, width, height;
        std::vector<uchar> rBinaryDataO, lBinaryDataO;

             NxLibCommand collect(cmdCollectPattern);
             try
             {
                collect.parameters()[itmBuffer] = false;
                collect.execute();
             }
             catch(NxLibException &e)
             {
             }

             //Left Image
             NxLibItem leftImgO  = mCamera[itmImages][itmWithOverlay][itmLeft ];
             leftImgO.getBinaryDataInfo(&width, &height, 0, 0,0,0);
             leftImgO.getBinaryData(&returnCode, lBinaryDataO, 0);

             QImage lImageO(&lBinaryDataO[0], width, height, QImage::Format_RGB888);
             ui->leftImage->setPixmap(QPixmap::fromImage(lImageO, Qt::AutoColor));

             //Right Image
             NxLibItem rightImgO = mCamera[itmImages][itmWithOverlay][itmRight];
             rightImgO.getBinaryDataInfo(&width, &height, 0,0,0,0);
             rightImgO.getBinaryData(&returnCode, rBinaryDataO, 0);

             QImage rImageO(&rBinaryDataO[0], width, height, QImage::Format_RGB888);
             ui->rightImage->setPixmap(QPixmap::fromImage(rImageO, Qt::AutoColor));

    }
    catch(...)
    {

    }
}


void HandEyeCalibrateDialog::on_originPushButton_clicked()
{

    mSettings.setValue("Origin/X", ui->XLineEdit->text().toDouble());
    mSettings.setValue("Origin/Y", ui->YLineEdit->text().toDouble());
    mSettings.setValue("Origin/Z", ui->ZLineEdit->text().toDouble());
    mSettings.setValue("Origin/A", ui->ALineEdit->text().toDouble());
    mSettings.setValue("Origin/B", ui->BLineEdit->text().toDouble());
    mSettings.setValue("Origin/C", ui->CLineEdit->text().toDouble());

}


void HandEyeCalibrateDialog::on_getOriginPushButton_clicked()
{
    ui->XLineEdit->setText(mSettings.value("Origin/X",0).toString());
    ui->YLineEdit->setText(mSettings.value("Origin/Y",0).toString());
    ui->ZLineEdit->setText(mSettings.value("Origin/Z",0).toString());
    ui->ALineEdit->setText(mSettings.value("Origin/A",0).toString());
    ui->BLineEdit->setText(mSettings.value("Origin/B",0).toString());
    ui->CLineEdit->setText(mSettings.value("Origin/C",0).toString());
}

void HandEyeCalibrateDialog::on_parameter_changed()
{
    mPendingChanges = true;
    writeParameters();
}

void HandEyeCalibrateDialog::writeParameters()
{
    if (!mPendingChanges) return;

    NxLibItem root; // Reference to the API tree root

    // Create an object referencing the camera's tree item, for easier access:
    NxLibItem captureParams = root[itmCameras][itmBySerialNo][0][itmParameters][itmCapture];
        captureParams[itmAutoExposure] = true;
    captureParams[itmAutoExposure] = ui->autoExposureChk->isChecked();
    captureParams[itmExposure] = ui->exposureSlider->value();

    mPendingChanges = false;
}
