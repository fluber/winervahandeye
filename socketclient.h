#ifndef SOCKETCLIENT_H
#define SOCKETCLIENT_H

#include <QTcpSocket>
#include <QObject>

class SocketClient : public QTcpSocket
{
    Q_OBJECT
public:
    explicit SocketClient(QObject *parent = nullptr);
    void setKey(qintptr);
    qintptr Key();
signals:
    void data_received(QString, int);
    void disconnected(qintptr);
public slots:
    void on_data_received();
    void on_disconnected();
private:
    qintptr mKey;
};

#endif // SOCKETCLIENT_H
