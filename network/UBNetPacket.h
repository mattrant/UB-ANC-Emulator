#ifndef UBNETPACKET_H
#define UBNETPACKET_H

#include <QObject>

class UBNetPacket : public QObject
{
    Q_OBJECT
public:
    explicit UBNetPacket(QObject *parent = 0);
    quint8 m_srcID;
    quint8 m_desID;

    QByteArray m_payload;


signals:

public slots:
    void setSrcID(quint8 srcID) {m_srcID = srcID;}
    void setDesID(quint8 desID) {m_desID = desID;}
    void setPayload(const QByteArray& payload) {m_payload = payload;}

public:
    quint8 getSrcID(void) {return m_srcID;}
    quint8 getDesID(void) {return m_desID;}
    QByteArray getPayload(void) {return m_payload;}

    QByteArray packetize(void);
    void depacketize(const QByteArray &packet);
};

#endif // UBNETPACKET_H
