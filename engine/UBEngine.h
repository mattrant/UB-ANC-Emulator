#ifndef UBENGINE_H
#define UBENGINE_H

#include <QObject>
#include <QVector>
#include <mutex>

class QTimer;

class UBObject;
class UASInterface;

class NS3Engine;

class UBEngine : public QObject
{
    Q_OBJECT
public:
    explicit UBEngine(QObject *parent = 0);
    void removeFromSwarm(int uav_id);

protected:
    QTimer* m_timer;
    QVector<UBObject*> m_objs;

    NS3Engine* m_ns3;

signals:

public slots:
    void startEngine();

    void networkEvent(UBObject *obj, const QByteArray &data);
    void positionChangeEvent(UASInterface *uav);

protected slots:
    void engineTracker(void);

    void uavAddedEvent(UASInterface* uav);

protected:
    double distance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2);
private:
    std::mutex instance_lock;
};

#endif // UBENGINE_H
