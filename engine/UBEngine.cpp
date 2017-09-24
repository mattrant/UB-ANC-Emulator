#include "UBEngine.h"
#include "UBObject.h"
#include "config.h"
#include "QsLog.h"

#include <QTimer>
#include <qmath.h>
#include <mutex>
#include <QtGlobal>
#include <QtMath>


#include "TCPLink.h"
#include "Waypoint.h"

#include "UASManager.h"
#include "LinkManager.h"
#include "UASWaypointManager.h"

#include "mercatorprojection.h"
#include "ArduPilotMegaMAV.h"
#include "NS3Engine.h"
#include "climits"
#include <fstream>
#include <unistd.h>
#include "UAS.h"
/**
 * @brief UBEngine::UBEngine
 * @param parent
 */
UBEngine::UBEngine(QObject *parent) : QObject(parent)
{
    m_timer = new QTimer(this);
    m_timer->setInterval(ENGINE_TRACK_RATE);

    //currently unused
    connect(m_timer, SIGNAL(timeout()), this, SLOT(engineTracker()));

    connect(UASManager::instance(), SIGNAL(UASCreated(UASInterface*)), this, SLOT(uavAddedEvent(UASInterface*)));

    m_ns3 = new NS3Engine(this);

}
/**
 * @brief UBEngine::engineTracker
 * Can be called repeatedly to perform some action on the drones
 */

void UBEngine::engineTracker(void){
//    if(m_objs.size()>=2&&m_objs[1]->getUAV()){
//       ArduPilotMegaMAV* uav = qobject_cast<ArduPilotMegaMAV*>(m_objs[1]->getUAV());
//      if( uav->getGroundSpeed() > 3){
//          removeFromSwarm(uav->getUASID());
//      }
//    }
//    for(int i =0;i<m_objs.size();i++){
//        QLOG_INFO()<<m_objs[i]->m_id;
//    }
}

/**
 * @brief UBAgent::init_task_locations
 * Generates random locations for a set number of tasks
 * @param seed number used to get consistent randomness between agents
 */
void write_task_locations(unsigned seed){
    std::ofstream wp_file;
    wp_file.open("wp_file.txt");
    wp_file<<"QGC WPL 110"<<std::endl;

    qsrand(seed);

    int num_tasks = 0;
    std::ifstream task_number;
    task_number.open("num_tasks.txt");
    task_number>>num_tasks;
    task_number.close();

    for(int i =0;i<num_tasks;++i){
        double base_lat = 43.000755; //field near UB North
        double base_lon = -78.776023;

        int rand_dist = qrand()%300;
        double rand_angle = qrand()%361;
        rand_angle = qDegreesToRadians(rand_angle);

        projections::MercatorProjection proj;
        core::Point pix1 = proj.FromLatLngToPixel(base_lat, base_lon, GND_RES);

        Vector3d v1(pix1.X(), pix1.Y(), 0);
        Vector3d v2(rand_dist*qCos(rand_angle),rand_dist*qSin(rand_angle),0);

        Vector3d v = v1+v2;

        internals::PointLatLng pll = proj.FromPixelToLatLng(v.x(), v.y(), GND_RES);

        double lat = pll.Lat();
        double lon = pll.Lng();
        wp_file<<i<<"\t0\t3\t16\t0\t5\t0\t0\t"<<lat<<"\t"<<lon<<"\t10\t1"<<std::endl;

    }
    wp_file.close();

}

/**
 * @brief UBEngine::startEngine
 *
 * Creates instances of all the MAV objects that are present as directories in the OBJECTS_PATH. Will load any mission config
 * file with the name determined by the MISSION_FILE macro
 */
void UBEngine::startEngine() {
    QDir dir(OBJECTS_PATH);
    uint seed = QDateTime::currentMSecsSinceEpoch()%UINT_MAX;
    //uint seed = 2324050835 ;
    QLOG_INFO()<<"Seed: "<<seed;
    qsrand(seed);
    write_task_locations(seed);


    dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
    int _instance = 0;
    foreach (QString folder, dir.entryList()) {
        QLOG_INFO()<<"Instantiating agent...";
        //instance_lock.lock();
        UBObject* obj = new UBObject(this,_instance+1);
        //needed so that the builders can send to each other as early as possible
        connect(obj, SIGNAL(netDataReady(UBObject*,QByteArray)), m_ns3, SLOT(networkEvent(UBObject*,QByteArray)));

        //generating random location for home for UAVs to test
//        double base_lat = 43.000755; //field near UB North
//        double base_lon = -78.776023;

//        int rand_dist = qrand()%50;
//        double rand_angle = qrand()%361;
//        rand_angle = qDegreesToRadians((double)rand_angle);

//        projections::MercatorProjection proj;
//        core::Point pix1 = proj.FromLatLngToPixel(base_lat, base_lon, GND_RES);

//        Vector3d v1(pix1.X(), pix1.Y(), 0);
//        Vector3d v2(rand_dist*qCos(rand_angle),rand_dist*qSin(rand_angle),0);

//        Vector3d v = v1+v2;

//        internals::PointLatLng pll = proj.FromPixelToLatLng(v.x(), v.y(), GND_RES);

//        double lat = pll.Lat();
//        double lon = pll.Lng();
//        double lat =base_lat;
//        double lon = base_lon;

        double lat;
        double lon;

        switch(_instance){
            case 0:
                lat = 43.0089759949898891;
                lon = -78.7899863719940186;
                break;
            case 1:
                lat = 43.0088740043467226;
                lon = -78.7898388504981995;
                break;
            case 2:
                lat = 43.0087720135341982;
                lon = -78.7899944186210632;
                break;
        }

        UASWaypointManager wpm;
        wpm.loadWaypoints(QString(OBJECTS_PATH) + QString("/") + folder + QString(MISSION_FILE));

        if (wpm.getWaypointEditableList().count()) {
            const Waypoint* wp = wpm.getWaypoint(0);

            lat = wp->getLatitude();
            lon = wp->getLongitude();
        }

        int port = MAV_PORT + _instance * 10;
        QString path = QString(OBJECTS_PATH) + QString("/") + folder;
        QStringList args;
        args << QString("--home") << QString::number(lat, 'f', 9) + "," + QString::number(lon, 'f', 9) + QString(",0,0") << QString("--model") << QString("quad") << QString("--instance") << QString::number(_instance);

        obj->setFirmware(path, args);

        path = QString(OBJECTS_PATH) + QString("/") + folder;
        args.clear();
        args << QString("--port") << QString::number(port);

        obj->setAgent(path, args);
        obj->startObject(port);

        m_objs.append(obj);
        _instance++;
        //instance_lock.unlock();
    }

    m_ns3->startEngine(&m_objs);

    m_timer->start();
}
/**
 * @brief UBEngine::uavAddedEvent
 * @param uav
 */
void UBEngine::uavAddedEvent(UASInterface* uav) {
    if (!uav)
        return;    


    connect(uav, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(positionChangeEvent(UASInterface*)));
    connect(uav, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), m_ns3, SLOT(positionChangeEvent(UASInterface*)));

    TCPLink* link = dynamic_cast<TCPLink*>(uav->getLinks()->first());

    if (!link)
        return;

    int i = link->getPort() - MAV_PORT;

    QDir dir(OBJECTS_PATH);
    dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);

    uav->getWaypointManager()->loadWaypoints(QString(OBJECTS_PATH) + QString("/") + dir.entryList()[i / 10] + QString(MISSION_FILE));
    uav->getWaypointManager()->writeWaypoints();

    UBObject* obj = m_objs[i / 10];
    ArduPilotMegaMAV* _uav = qobject_cast<ArduPilotMegaMAV*>(uav);
    while(uav->getUASID()!=obj->m_id){
        //QLOG_INFO()<<"Current MAV ID: "<<uav->getUASID();
        //_uav->setParameter(1,"SYSID_THISMAV",obj->m_id);
       // UAS* uas = qobject_cast<UAS*>(uav);
        _uav->setParameter(1,"SYSID_THISMAV",QVariant(obj->m_id));
        sleep(1);

    }
     QLOG_INFO()<<"***Changing ID to "<<obj->m_id <<"| Actual: "<<uav->getUASID();
    obj->setUAV(uav);
    obj->setCR(COMM_RANGE);
    obj->setVR(VISUAL_RANGE);
    //connect(obj, SIGNAL(netDataReady(UBObject*,QByteArray)), this, SLOT(networkEvent(UBObject*,QByteArray)));
    connect(obj, SIGNAL(netDataReady(UBObject*,QByteArray)), m_ns3, SLOT(networkEvent(UBObject*,QByteArray)));
    QLOG_INFO() << "New UAV Connected | MAV ID: " << uav->getUASID();


}
/**
 * @brief UBEngine::positionChangeEvent
 * Slot is run whenever an agent has changed its coordinates. Sends new sensor infor to drones that are within a certain distance
 * @param uav
 */
void UBEngine::positionChangeEvent(UASInterface* uav) {
   // QLOG_INFO()<<"PositionChanged | "<<uav->getUASID()<<" |"<<uav->getLatitude()<<" "<<uav->getLongitude()<<" "<<uav->getAltitudeAMSL();
    if (!uav)
        return;

    UBObject* obj = NULL;
    foreach(UBObject* _obj, m_objs) {
        if (_obj->getUAV() == uav) {
            obj = _obj;
            break;
        }
    }

    if (!obj)
        return;

    foreach (UBObject* _obj, m_objs) {
        UASInterface* _uav = _obj->getUAV();
        if (!_uav)
            continue;

        if (_uav == uav)
            continue;

        char data[2];
        double dist = distance(uav->getLatitude(), uav->getLongitude(), uav->getAltitudeAMSL(), _uav->getLatitude(), _uav->getLongitude(), _uav->getAltitudeAMSL());

        data[0] = _obj->getUAV()->getUASID();
        data[1] = false;

        if (dist < obj->getVR())
            data[1] = true;

        obj->snrSendData(QByteArray(data, 2));

        data[0] = obj->getUAV()->getUASID();
        data[1] = false;

        if (dist < _obj->getVR())
            data[1] = true;

        _obj->snrSendData(QByteArray(data, 2));
    }

    foreach (Waypoint* wp, uav->getWaypointManager()->getWaypointEditableList()) {
        if (wp->getAction() != MAV_CMD_DO_SET_ROI)
            continue;

        char data[2];
        double dist = distance(uav->getLatitude(), uav->getLongitude(), uav->getAltitudeRelative(), wp->getLatitude(), wp->getLongitude(), wp->getAltitude());

        data[0] = wp->getId();
        data[1] = false;

        if (dist < obj->getVR())
            data[1] = true;

        obj->snrSendData(QByteArray(data, 2));
    }
}
/**
 * @brief UBEngine::networkEvent
 * Slot is run if data is being sent across the network. Currently, nodes within a specified
 * radius of the sending node will all recieve the data being sent
 * @param obj object that is sending the data out
 * @param data
 */
void UBEngine::networkEvent(UBObject* obj, const QByteArray& data) {
    QLOG_INFO()<<m_objs.size()<<"| Event...";
    UASInterface* uav = obj->getUAV();
    if (!uav)
        return;

    /*
     * For all nodes in the netork, check if they are withing the comms radius of the
     * sender. If they are within that distance, send them the info
     * */

    foreach (UBObject* _obj, m_objs) {
        UASInterface* _uav = _obj->getUAV();
        if (!_uav)
            continue;

//        if (_uav == uav)
//            continue;


        double dist = distance(uav->getLatitude(), uav->getLongitude(), uav->getAltitudeAMSL(), _uav->getLatitude(), _uav->getLongitude(), _uav->getAltitudeAMSL());
        //change this to a more realistic model for communication
        if (dist < obj->getCR())
            _obj->netSendData(data);
    }
}
/**
 * @brief UBEngine::distance
 * Calculates the distances between two 3-tuples specified by (lat,long,alt)
 * @param lat1
 * @param lon1
 * @param alt1
 * @param lat2
 * @param lon2
 * @param alt2
 * @return double: the distance between the two points
 */

double UBEngine::distance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2) {
   double x1, y1, z1;
   double x2, y2, z2;

   projections::MercatorProjection proj;

   proj.FromGeodeticToCartesian(lat1, lon1, alt1, x1, y1, z1);
   proj.FromGeodeticToCartesian(lat2, lon2, alt2, x2, y2, z2);

   return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}
/**
 * @brief removeFromSwarm
 * Removes the specified uav from the swarm. The uav processes will be killed and it will appear that the drone has simply dissappeared. For
 * all intents and purposes the drone no longer exists
 * @param uav_id ID of the uav that should be removed from the swarm.
 */
void UBEngine::removeFromSwarm(int uav_id){

    for(int i =0;i<m_objs.size();++i){

        UBObject* obj = m_objs[i];
        UASInterface* curr_uav = obj->getUAV();

        if(curr_uav->getUASID()==uav_id){
            QLOG_INFO()<<"Removing uav from swarm | ID "<<uav_id;
            obj->killUAV();
            m_objs.erase(m_objs.begin()+i);
            m_ns3->bringDownNode(curr_uav);
            break;
        }
    }
}
