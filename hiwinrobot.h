#ifndef HIWINROBOT_H
#define HIWINROBOT_H

#include <QTcpServer>
#include <QObject>
#include <QList>

class SocketClient;

class HiWinRobot : public QTcpServer
{
    Q_OBJECT
    Q_PROPERTY(bool moveDone READ moveDone WRITE setMoveDone NOTIFY moveDoneChanged)
public:
    explicit HiWinRobot(QObject *parent = nullptr);
    ~HiWinRobot();

    QString getPose();
    void start();
    void stop();
    void move(double x, double y, double z,
              double a, double b, double c);
    void setMoveDone(bool moveDone)
    {
        mMoveDone = moveDone;
        emit moveDoneChanged(moveDone);
    }

    bool moveDone() const
    {return mMoveDone; }
signals:
    void client_data_received(QString, int);
    void server_data_sended(QString);
    void moveDoneChanged(bool);
public slots:
    void on_client_data_received(QString, int);
    void on_client_disconnected(qintptr);
protected:
    void incomingConnection(qintptr);
private:
    uint mServerPort;
    QList<SocketClient *> *mClientList = nullptr;
    bool mMoveDone = false;
};

#endif // HIWINROBOT_H
