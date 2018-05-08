#include "NS3Engine.h"
#include "UBObject.h"
#include "UBNetPacket.h"

#include <qtconcurrentrun.h>

#include "UASInterface.h"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("UB-ANC-Emulator");

#define PXY_PORT    45760
#define PACKET_END  "\r\r\n\n"

NS3Engine::NS3Engine(QObject *parent) : QObject(parent)
{

}

void NS3Engine::startEngine(QVector<UBObject*>* objs) {
    m_objs = objs;

    QFuture<void> ns3 = QtConcurrent::run(this, &NS3Engine::startNS3);
//    ns3.waitForFinished();
}
/**
 * @brief NS3Engine::startNS3 Initializes and starts the NS3 discrete event simulator. Any changes to the configuration of the NS3 simulator should
 * be made inside of this function
 */
void NS3Engine::startNS3() {
    GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    m_nodes.Create(m_objs->size());

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (int i = 0; i < m_objs->size(); i++) {
        positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    }

    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(m_nodes);

    std::string phyMode("DsssRate1Mbps");
    bool verbose = false;
    bool tracing = true;

    // disable fragmentation for frames below 2200 bytes
    Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));
    // turn off RTS/CTS for frames below 2200 bytes
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("2200"));
    // Fix non-unicast data rate to be the same as that of unicast
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

    // The below set of helpers will help us to put together the wifi NICs we want
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");

    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-110.0) );
    wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-110.0) );
    wifiPhy.Set ("TxPowerStart", DoubleValue (15.0) );
    wifiPhy.Set ("TxPowerEnd", DoubleValue (15.0) );
    wifiPhy.Set ("RxGain", DoubleValue (-30) );
    wifiPhy.Set ("RxNoiseFigure", DoubleValue (7) );
    // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
    wifiPhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
    wifiPhy.SetChannel(wifiChannel.Create());

    // Add an upper mac and disable rate control
    WifiMacHelper wifiMac;
    // Set it to adhoc mode
    wifiMac.SetType("ns3::AdhocWifiMac");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode), "ControlMode", StringValue(phyMode));
    if (verbose) {
        wifi.EnableLogComponents();  // Turn on all Wifi logging
    }

    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, m_nodes);

    // Enable OLSR
    OlsrHelper olsr;
    AodvHelper aodv;
    Ipv4StaticRoutingHelper staticRouting;

    Ipv4ListRoutingHelper list;
    list.Add(staticRouting, 0);
    //list.Add(olsr, 10);
    //list.Add(aodv, 0);

    InternetStackHelper internet;
    internet.SetRoutingHelper(list); // has effect on the next Install()
    internet.Install(m_nodes);

    Ipv4AddressHelper ipv4;
    // Assign IP Addresses
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devices);

    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), PXY_PORT);
    uint16_t sinkPort = 8080;
    for (Ipv4InterfaceContainer::Iterator it = i.Begin(); it != i.End(); ++it) {
          Ptr<Node> node = it->first->GetNetDevice(it->second)->GetNode();
          //UBObject* obj = m_nods->value(GetPointer(node));
          Ipv4Address addr = it->first->GetAddress(it->second, 0).GetLocal();

          PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory",InetSocketAddress(addr,sinkPort));
          ApplicationContainer sinkApps = packetSinkHelper.Install(node);
          sinkApps.Start(Seconds(0.));
          sinkApps.Get(0)->TraceConnectWithoutContext("Rx",MakeCallback(&NS3Engine::receivePacket,this));

    }
    if (tracing == true) {
        AsciiTraceHelper ascii;
        wifiPhy.EnableAsciiAll(ascii.CreateFileStream("wifi-adhoc.tr"));
        wifiPhy.EnablePcap("wifi-adhoc", devices);
        // Trace routing tables
        Ptr<OutputStreamWrapper> olsrRoutingStream = Create<OutputStreamWrapper>("olsr.routes", std::ios::out);
        olsr.PrintRoutingTableAllEvery(Seconds(2), olsrRoutingStream);
        Ptr<OutputStreamWrapper> olsrNeighborStream = Create<OutputStreamWrapper>("olsr.neighbors", std::ios::out);
        olsr.PrintNeighborCacheAllEvery(Seconds(2), olsrNeighborStream);

        Ptr<OutputStreamWrapper> aodvRoutingStream = Create<OutputStreamWrapper>("aodv.routes", std::ios::out);
        aodv.PrintRoutingTableAllEvery(Seconds(2), aodvRoutingStream);
        Ptr<OutputStreamWrapper> aodvNeighborStream = Create<OutputStreamWrapper>("aodv.neighbors", std::ios::out);
        aodv.PrintNeighborCacheAllEvery(Seconds(2), aodvNeighborStream);
    }



    Simulator::Run();
    Simulator::Destroy();
}

void NS3Engine::receivePacket(Ptr<const Packet> p,const Address& from) {


    //extract destination info to determine object to receive packet
    QByteArray info(p->GetSize(), 0);
    p->CopyData((uint8_t*)info.data(), p->GetSize());
    UBNetPacket ubp;
    ubp.depacketize(info);


    quint8 possible = 3;
    //std::cout<<"Received by "<<QString::number(ubp.getDesID()).toStdString()<<std::endl;
    for(int i =0;i<m_objs->size();++i){
      if(m_objs->at(i)->m_id==ubp.getDesID()){
        QMetaObject::invokeMethod(m_objs->at(i), "netSendData", Qt::QueuedConnection, Q_ARG(const QByteArray&, info));
        break;
      }
    }












}

/**
 * @brief NS3Engine::networkEvent
 * Slot will run anytime an agent wishes to send information across the network. The event will be scheduled within the NS3 simulation engine
 * @param obj
 * @param data
 */
void NS3Engine::networkEvent(UBObject* obj, const QByteArray& data) {
    static QVector<QByteArray> vdata(m_objs->size());

    uint32_t index = m_objs->indexOf(obj);
    if (index == -1)
        return;

    vdata[index] += data;

    while (vdata[index].contains(PACKET_END)) {
        int bytes = vdata[index].indexOf(PACKET_END);

        UBNetPacket packet;
        packet.depacketize(vdata[index].left(bytes));

        if(packet.getDesID()==255){

            for(int i =0;i<m_objs->size();++i){
                //skip sending it to sender
                if(m_objs->at(i)->m_id == obj->m_id){
                    continue;
                }

                packet.m_desID = m_objs->at(i)->m_id;
                InetSocketAddress remote = InetSocketAddress(Ipv4Address(tr("10.1.1.%1").arg(packet.getDesID()).toStdString().c_str()), 8080);
            //        InetSocketAddress remote = InetSocketAddress(Ipv4Address("255.255.255.255"), PXY_PORT);

                QByteArray pkt = packet.packetize().append(PACKET_END);
                //std::cout<<QString::number(packet.getDesID()).toStdString()<< " "<<QString::number(packet.getSrcID()).toStdString()<<"Contents: " <<pkt.toHex().toStdString().c_str()<<std::endl;

            //        Simulator::ScheduleWithContext(m_nodes.Get(index)->GetId(), Time(0), &Socket::SendTo, socket, (uint8_t*)pkt.data(), pkt.size(), 0, remote);
                Ptr<Socket> socket = ns3::Socket::CreateSocket(m_nodes.Get(index),ns3::TcpSocketFactory::GetTypeId());
                Simulator::ScheduleWithContext(m_nodes.Get(index)->GetId(), Time(0), &NS3Engine::sendData, this, socket, remote, pkt);
            }
        }
        else{
            InetSocketAddress remote = InetSocketAddress(Ipv4Address(tr("10.1.1.%1").arg(packet.getDesID()).toStdString().c_str()), 8080);
        //        InetSocketAddress remote = InetSocketAddress(Ipv4Address("255.255.255.255"), PXY_PORT);

            QByteArray pkt = packet.packetize().append(PACKET_END);

        //        Simulator::ScheduleWithContext(m_nodes.Get(index)->GetId(), Time(0), &Socket::SendTo, socket, (uint8_t*)pkt.data(), pkt.size(), 0, remote);
            Ptr<Socket> socket = ns3::Socket::CreateSocket(m_nodes.Get(index),ns3::TcpSocketFactory::GetTypeId());
            Simulator::ScheduleWithContext(m_nodes.Get(index)->GetId(), Time(0), &NS3Engine::sendData, this, socket, remote, pkt);
        }

        vdata[index] = vdata[index].mid(bytes + qstrlen(PACKET_END));
    }
}





void NS3Engine::sendData(Ptr<Socket> socket, const Address& remote, const QByteArray& packet) {
    socket->Bind();

    if(socket->Connect(remote)==0){
      //std::cout<<"Successfully connected to "<<remote<<std::endl;
      socket->Send((uint8_t*)packet.data(), packet.size(), 0);
    }
    socket->Close();
}

void NS3Engine::positionChangeEvent(UASInterface* uav) {
    if (!uav)
        return;

    double lat = uav->getLatitude();
    double lon = uav->getLongitude();
    double alt = uav->getAltitudeAMSL();
    Vector pos = GeographicPositions::GeographicToCartesianCoordinates(lat, lon, alt, GeographicPositions::WGS84);

    uint32_t index = -1;
    for (int i = 0; i < m_objs->size(); i++) {
        if (m_objs->at(i)->getUAV() == uav) {
            index = i;
            break;
        }
    }

    if (index == -1)
        return;

    Ptr<MobilityModel> mm = m_nodes.Get(index)->GetObject<MobilityModel>();

//    Simulator::ScheduleNow(&MobilityModel::SetPosition, mm, pos);
    Simulator::ScheduleWithContext(m_nodes.Get(index)->GetId(), Time(0), &MobilityModel::SetPosition, mm, pos);
}

void NS3Engine::bringDownNode(UASInterface* uav){
    int index = 0;
    for (int i = 0; i < m_objs->size(); i++) {
        if (m_objs->at(i)->getUAV() == uav) {
            index = i;
            break;
        }
    }
    //brings down the interface for the specified node so that the interface willnot be considered during IPV4 routing
   Ptr<Ipv4> ipv4 =  m_nodes.Get(index)->GetDevice(0)->GetNode()->GetObject<Ipv4>();
   ipv4->SetDown(ipv4->GetInterfaceForDevice(m_nodes.Get(index)->GetDevice(0)));
}
