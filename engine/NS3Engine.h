#ifndef NS3ENGINE_H
#define NS3ENGINE_H

#include <QObject>
#include <QVector>

#include "ns3/socket.h"
#include "ns3/node-container.h"

class UBObject;
class UASInterface;

class NS3Engine : public QObject
{
    Q_OBJECT
public:
    explicit NS3Engine(QObject *parent = 0);

    void startEngine(QVector<UBObject*> *objs);
    void startNS3();
    void receivePacket(ns3::Ptr<ns3::Socket> socket);
    void bringDownNode(UASInterface* uav);

signals:

public slots:
    void networkEvent(UBObject* obj, const QByteArray& data);
    void sendData(ns3::Ptr<ns3::Socket> socket, const ns3::Address &remote, const QByteArray& packet);

    void positionChangeEvent(UASInterface* uav);

protected:
    QVector<UBObject*>* m_objs;

    ns3::NodeContainer m_nodes;
};

#endif // NS3ENGINE_H
