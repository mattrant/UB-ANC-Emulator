#ifndef UBOBJECT_H
#define UBOBJECT_H

#include <QObject>

#include <QQueue>
#include <QByteArray>
#include <QAbstractSocket>

#include "UBServer.h"

class QTimer;
class QProcess;
class QTcpSocket;

class UASInterface;

class UBObject : public QObject
{
    Q_OBJECT
public:
    explicit UBObject(QObject *parent = 0);

protected:
    UASInterface* m_uav;

    quint16 m_port;

    quint32 m_cr;
    quint32 m_vr;

    QProcess* m_firmware;
    QProcess* m_agent;

    UBServer* m_net_server;
    UBServer* m_snr_server;

    QTcpSocket* m_socket;

    QTimer* m_timer;

signals:
    void netDataReady(UBObject* obj, const QByteArray& data);
    void snrDataReady(UBObject* obj, const QByteArray& data);

protected slots:
    void objectTracker(void);

    void netClientConnectedEvent(quint16 port);
    void netDataReadyEvent(const QByteArray& data) { emit netDataReady(this, data); }

    void snrClientConnectedEvent(quint16 port);
    void snrDataReadyEvent(const QByteArray& data) { emit snrDataReady(this, data); }

    void connectedEvent();
    void disconnectedEvent();
    void errorEvent(QAbstractSocket::SocketError err);

public slots:
    void setUAV(UASInterface* uav) {m_uav = uav;}

    void setCR(quint32 cr) {m_cr = cr;}
    void setVR(quint32 vr) {m_vr = vr;}

    void netSendData(const QByteArray& data) {m_net_server->sendData(data);}
    void snrSendData(const QByteArray& data) {m_snr_server->sendData(data);}

    void setFirmware(const QString& path, const QStringList& args);
    void setAgent(const QString& path, const QStringList& args);

    void startObject(int port);

public:
    UASInterface* getUAV(void) {return m_uav;}

    quint32 getCR(void) {return m_cr;}
    quint32 getVR(void) {return m_vr;}
    void killUAV();
};

#endif // UBOBJECT_H
