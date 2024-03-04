#include "socketclient.h"
#include <QDebug>

SocketClient::SocketClient(QObject *parent) : QTcpSocket(parent)
{
   connect(this, SIGNAL(readyRead()), this, SLOT(on_data_received()));
   connect(this, SIGNAL(disconnected()), this, SLOT(on_disconnected()));
}

void SocketClient::setKey(qintptr key)
{
    mKey = key;
    setSocketDescriptor(key);
    qDebug() << "Client Key: " << mKey;
    qDebug() << "Client SocketDescriptor: " << socketDescriptor();
}

qintptr SocketClient::Key()
{
    return mKey;
}

void SocketClient::on_data_received()
{
    int length = bytesAvailable();
    if (length  > 0)
    {
        char buf[1024] = "";
        read(buf, length);

        qDebug() << "R:" << QString(buf);

        emit data_received(QString(buf), length);
    }
}

void SocketClient::on_disconnected()
{
    emit disconnected(mKey);
}
