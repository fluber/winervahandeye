#include "hiwinrobot.h"
#include <QSettings>
#include <QTcpServer>
#include <QException>
#include <QTcpSocket>
#include <QDebug>
#include "socketclient.h"
HiWinRobot::HiWinRobot(QObject *parent)
    : QTcpServer(parent),
      mClientList(new QList<SocketClient*>())
{
    QSettings settings;
    mServerPort = settings.value("HiWinRobot/ServerPort", 4000).toUInt();

}

HiWinRobot::~HiWinRobot()
{
    delete mClientList;
}



QString HiWinRobot::getPose()
{
    if (!this->isListening())
        return "";
    if (mClientList->isEmpty())
        return "";
    try
    {
        QString cmd("[2,0,0,0,0,0,0,0]");
        for (int i = 0; i < mClientList->count(); i++) {
            SocketClient* client = mClientList->at(i);
            if (client->isWritable())
            {
                client->write(cmd.toLatin1(), cmd.length());
            }
        }
        emit server_data_sended(cmd);
        return "";
    }
    catch (const QString& e)
    {
        qDebug() << "Exception: " << e;
        return "";
    }
    catch (...)
    {
        return "";
    }
}

void HiWinRobot::start()
{
    if (!listen(QHostAddress::Any, mServerPort))
    {
        qDebug() << "Could not start server: " << this->errorString();
        throw this->errorString();
    }
    else
    {
        qDebug() << "Listening to port " << mServerPort << "...";
    }
}

void HiWinRobot::stop()
{
    close();
    qDebug() << "stop listen";
    mClientList->clear();
}

void HiWinRobot::move(double x, double y, double z, double a, double b, double c)
{
    if (!this->isListening())
        return;
    if (mClientList->isEmpty())
        return;
    try
    {
        this->setMoveDone(false);
        QString patern("[1,0,%1,%2,%3,%4,%5,%6]");
        QString cmd = patern.arg(x).arg(y).arg(z).arg(a).arg(b).arg(c);
        for (int i = 0; i < mClientList->count(); i++) {
            SocketClient* client = mClientList->at(i);
            if (client->isWritable())
            {
                client->write(cmd.toLatin1(), cmd.length());
            }
        }
        emit server_data_sended(cmd);
    }
    catch (const QString& e)
    {
        qDebug() << "Exception: " << e;
    }
    catch (...)
    {

    }
}

void HiWinRobot::on_client_data_received(QString message, int length)
{
    emit client_data_received(message, length);
}

void HiWinRobot::on_client_disconnected(qintptr descriptor)
{
    qDebug() << "on_client_disconnected: " << descriptor;
    for (int i = 0; i < mClientList->count(); i++)
    {
        if (mClientList->at(i)->Key() == descriptor)
        {
            qDebug() << "Remove Client: " << descriptor;
            mClientList->removeAt(i);
            return;
        }
    }


}

void HiWinRobot::incomingConnection(qintptr handle)
{
    qDebug() << "IncomingConnection: " << handle;
    SocketClient *client = new SocketClient(this);
    connect(client, SIGNAL(data_received(QString,int)), this, SLOT(on_client_data_received(QString,int)));
    connect(client, SIGNAL(disconnected(qintptr)), this, SLOT(on_client_disconnected(qintptr)));
    client->setKey(handle);
    mClientList->clear(); //Only One Client
    mClientList->append(client);

}

