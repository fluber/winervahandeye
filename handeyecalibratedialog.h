#ifndef HANDEYECALIBRATEDIALOG_H
#define HANDEYECALIBRATEDIALOG_H

#include <QDialog>
#include <QQueue>
#include <QSettings>
#include <QTimer>
#include "nxLib.h"
#include <QLabel>
#include <QSlider>

class HiWinRobot;
class QListWidgetItem;

namespace Ui {
class HandEyeCalibrateDialog;
}

struct RobotPoseStruct {
    double x;
    double y;
    double z;
    double a;
    double b;
    double c;
};

class HandEyeCalibrateDialog : public QDialog
{
    Q_OBJECT

public:
    explicit HandEyeCalibrateDialog(QWidget *parent = 0);
    ~HandEyeCalibrateDialog();
private:
    void offline();
    void onLine();
    void parseRobotPose(QString message, double &x, double &y, double &z, double &a, double &b, double &c);
    void loadPoses();
    QPixmap convertXYZBuffer(std::vector<float> const& depthBuffer, int bufferWidth, int bufferHeight);
    unsigned char valueToColorIndex(float z, float offset, float zColorRepetitionDistance);
    QPixmap imageToPixmap(QImage& image, int width = -1, int height = -1, int x = 0, int y = 0);
    void updateLabelAndSlider(bool forceSliderPosition, QLabel* label, QSlider* slider, double inputValue, double scale = 1, double specialValue = std::numeric_limits<double>::quiet_NaN(), std::string const& specialText = "", double factor = 1.0);
private slots:
    void on_SocketButton_clicked();

    void on_readParameters(bool forceSliderPositions = false);

    void on_MoveButton_clicked();

    void on_GetPoseButton_clicked();

    void on_client_data_received(QString, int);

    void on_server_data_sended(QString);

    void on_robotPoseListWidget_itemDoubleClicked(QListWidgetItem *item);

    void on_poseUpPushButton_clicked();

    void on_poseDownPushButton_clicked();

    void on_removePosePushButton_clicked();

    void on_clearPosePushButton_clicked();

    void on_MovePushButton_clicked();

    void on_loadRobotPosePushButton_clicked();

    void on_saveRobotPosePushButton_clicked();

    void on_logListWidget_itemDoubleClicked(QListWidgetItem *item);

    void on_CalibratePushButton_clicked();

    void on_groupBox_3_clicked();

    void on_calibrateCheckBox_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_continueMovePushButton_clicked();

    void on_stopPushButton_clicked();

    void on_stopCalibratePushButton_clicked();

    void on_saveLogPushButton_clicked();

    void on_grabImageCheckBox_clicked(bool checked);

    void on_grab_Image();


    void on_originPushButton_clicked();

    void on_getOriginPushButton_clicked();
    void on_parameter_changed();
    void writeParameters();
private:
    Ui::HandEyeCalibrateDialog *ui;
    bool mOnLine = false;
    HiWinRobot *mHiWinRobot;
    bool mCalibrating = false;
    QQueue<RobotPoseStruct> mPoseQueue;
    bool mStopContinueMove;
    QSettings mSettings;
    bool mColorsSymmetric;
    static int const FixedColors = 3;
    float zMinValue, zMaxValue;
    float zColorRepetitionDistance;
    QVector<QRgb> mDepthColors;
    NxLibItem mRoot; // Reference to the API tree root
    NxLibItem mCamera;
    QTimer mGrabImageTimer;
    QTimer *mReadParametersTimer;
    bool mPendingChanges;
};

#endif // HANDEYECALIBRATEDIALOG_H
