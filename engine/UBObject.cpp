#include "UBObject.h"
#include "UBServer.h"

#include "config.h"

#include <QTimer>
#include <QProcess>
#include <QTcpSocket>

#include "UASInterface.h"

#include "LinkManager.h"
#include "LinkManagerFactory.h"
/**
 * @brief UBObject::UBObject
 * Instantiates all the servers used for the various parts of the drone. The actual configurations of all these
 * servers is done through the use of getter functions
 * @param parent
 */
UBObject::UBObject(QObject *parent) : QObject(parent),
    m_uav(NULL),
    m_cr(0),
    m_vr(0)
{
    m_firmware = new QProcess(this);
    m_agent = new QProcess(this);

    m_net_server = new UBServer(this);
    m_snr_server = new UBServer(this);

    connect(m_net_server, SIGNAL(clientConnected(quint16)), this, SLOT(netClientConnectedEvent(quint16)));
    connect(m_net_server, SIGNAL(dataReady(QByteArray)), this, SLOT(netDataReadyEvent(QByteArray)));

    connect(m_snr_server, SIGNAL(clientConnected(quint16)), this, SLOT(snrClientConnectedEvent(quint16)));
    connect(m_snr_server, SIGNAL(dataReady(QByteArray)), this, SLOT(snrDataReadyEvent(QByteArray)));

    m_socket = new QTcpSocket(this);

    connect(m_socket, SIGNAL(connected()), this, SLOT(connectedEvent()));
    connect(m_socket, SIGNAL(disconnected()), this, SLOT(disconnectedEvent()));
    connect(m_socket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(errorEvent(QAbstractSocket::SocketError)));

    m_timer = new QTimer(this);
    m_timer->setInterval(OBJECT_TRACK_RATE);

    connect(m_timer, SIGNAL(timeout()), this, SLOT(objectTracker()));
}

void UBObject::setFirmware(const QString& path, const QStringList& args) {
    m_firmware->setWorkingDirectory(path);
    m_firmware->setProgram(QString(FIRMWARE_FILE));
    m_firmware->setArguments(args);
}

void UBObject::setAgent(const QString& path, const QStringList& args) {
    m_agent->setWorkingDirectory(path);
    m_agent->setProgram(QString(AGENT_FILE));
    m_agent->setArguments(args);
}
/**
 * @brief UBObject::startObject
 * Starts all of the servers associated with the MAV Object
 * @param port
 */
void UBObject::startObject(int port) {
    m_port = port;

    m_firmware->start();

    m_net_server->startServer((PHY_PORT - MAV_PORT) + m_port);
    m_snr_server->startServer((SNR_PORT - MAV_PORT) + m_port);

    m_socket->connectToHost(QHostAddress::LocalHost, m_port);
}

void UBObject::objectTracker() {
}

void UBObject::connectedEvent() {
    m_socket->disconnectFromHost();
}

void UBObject::disconnectedEvent() {
    if (m_port % 10) {
        int link = LinkManagerFactory::addTcpConnection(QHostAddress::LocalHost, "", m_port, false);
        LinkManager::instance()->connectLink(link);

        m_timer->start();
    } else {
        m_port += 2;
        //needed so that stdout/stderr will print to parent stdout/stderr
        m_agent->setProcessChannelMode(QProcess::ForwardedChannels);
        m_agent->start();
        m_socket->connectToHost(QHostAddress::LocalHost, m_port);
    }
}

void UBObject::errorEvent(QAbstractSocket::SocketError err) {
    if (err == QAbstractSocket::ConnectionRefusedError) {
        m_socket->connectToHost(QHostAddress::LocalHost, m_port);
    }
}

void UBObject::netClientConnectedEvent(quint16 port) {
    QLOG_INFO() << "PHY Connected | Port: " << port;
}

void UBObject::snrClientConnectedEvent(quint16 port) {
    QLOG_INFO() << "SNR Connected | Port: " << port;
}

/**
 * @brief UBObject::killUAV
 * Simulates immediate failure of a uav
 */
void UBObject::killUAV(){
    m_agent->kill();
    m_firmware->kill();
}
