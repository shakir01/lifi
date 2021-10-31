#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/csma-module.h"
#include "ns3/config-store-module.h"
#include "ns3/net-device-container.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/lifi-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/lifi-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/lifi-constant-velocity-mobility-model.h"
#include "ns3/lifi-random-waypoint-mobility-model.h"
#include "ns3/lifi-user-device-infos.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/centralized-controller.h"
#include <unordered_map>
#include "ns3/gnuplot.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <list>
#include <cmath>
#include <string>

using namespace ns3;

double simulationTime = 10;                   /* Simulation time in seconds. */

class LiFiController : public Object
{
public:
  LiFiController ();
  ~LiFiController ();
  virtual void DoDispose();
  void RunSimulation (uint32_t nWifis, double totalTime, std::string rate);
  uint32_t m_lifi_stations;//User connected to LiFi and potentially to WiFi
  uint32_t m_wifi_stations;//Users connected only to wifi
  Ptr<FlowMonitor>  m_monitor;
  Ptr<CentralizedController> m_Centralized_Controller;
  NodeContainer m_wifinodesContainer;
  NodeContainer lifi_nodesContainer;
  NodeContainer server_nodesContainer;

  NetDeviceContainer wifiNetDevices;//contains lifi aps
  NetDeviceContainer lifiNetDevicesContainer;//net devices of LiFi AP and its connected user devices
  NetDeviceContainer lifiToCCNetDeviceContainer;//Link (here p2p but could be csma) connecting LiFi AP to WiFi AP/Centralized Controller (CC)
  NetDeviceContainer csmaServerNetDevices;//Net Devices of servers  containing source/destination 

  Ipv4InterfaceContainer serversIpv4interfaceContainer;
  Ipv4InterfaceContainer wifiIpv4interfaceContainer;
  Ipv4InterfaceContainer lifiAPIpv4interfaceContainer;
  Ipv4InterfaceContainer lifiAPtoCCInterfaceContainer;

  std::vector<std::vector<double> > m_position;
  void ReadUsersFromCSV (uint16_t num_nodes);

  void CreateNodes (uint32_t);
  void CreateController();

  void ConfigureLiFiNetwork ();
  void ConfigureWiFiNetwork ();
  void SetupMobility ();

  void InstallInternetStack ();
  void InstallApplications(std::string);

  void ConfigureServers ();

  void InstallLiFiNodesApplications (std::string dataRate);
  void MonitorLiFiRxChannelStats(Ptr<LiFiPhy>, uint32_t,std::vector<ns3::ChannelStats> channelStatsVector);
  void RunSimulation (uint32_t lifinodes, uint32_t wifinodes,double totalTime, std::string rate);
void CheckThroughput (FlowMonitorHelper* m_flowmon);
};

int main (int argc, char **argv)
{
    
    std::string dataRates = "0.2Mbps"; 
    uint16_t simTime = 30;
    Ptr<LiFiController> test = CreateObject<LiFiController> ();//nWifis// nSinks  

    uint32_t lifi_stations = 5;
    uint32_t wifi_stations = 0;
    test->RunSimulation (lifi_stations, wifi_stations,simTime, dataRates);
    return 0;
} 

LiFiController::LiFiController ()
{
}

LiFiController::~LiFiController ()
{
  
}

void LiFiController::DoDispose()
{
  m_monitor = 0;
 // DoDispose();
}

void LiFiController::RunSimulation (uint32_t lifinodes, uint32_t wifinodes,double totalTime, std::string rate)
{
  simulationTime = totalTime;
  m_lifi_stations = lifinodes;//User connected to LiFi and potentially to WiFi
  m_wifi_stations=wifinodes;//Users connected only to wifi
  CreateNodes(lifinodes);
  CreateController();
  ConfigureServers();
  ConfigureLiFiNetwork ();
  ConfigureWiFiNetwork ();
  InstallInternetStack ();
  InstallLiFiNodesApplications(rate);
  //InstallWiFiNodesApplications(rate);
  FlowMonitorHelper m_flowmon;
  SetupMobility ();
  m_monitor = m_flowmon.InstallAll ();
  Simulator::Stop (Seconds (simulationTime+1));
  Simulator::Run ();
  Simulator::Destroy ();
  CheckThroughput(&m_flowmon);
  //AnalyzeData(&m_flowmon, m_monitor,rate,nWifis);
}

void LiFiController::CheckThroughput (FlowMonitorHelper* m_flowmon)
{
   m_monitor->CheckForLostPackets ();
   Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (m_flowmon->GetClassifier ());
   FlowMonitor::FlowStatsContainer stats = m_monitor->GetFlowStats ();
   for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
     {
       Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);

       std::cout << "Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
       std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
       std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
       std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / 9.0 / 1000 / 1000  << " Mbps\n";
       std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
       std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
       std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / 9.0 / 1040 /1040  << " Mbps\n";
      // double throughput = i->second.rxBytes * 8.0 / 9.0 / 1040 / 1040;
     }

   m_flowmon->SerializeToXmlFile ("TdmaExample-Stats.xml", true , true);
}

void LiFiController::CreateNodes (uint32_t usernodes)
{
  uint32_t wifi = 1;
  uint32_t totallifinodes = m_lifi_stations+1;
  m_wifinodesContainer.Create (wifi);//we initially create only wifi AP node...user nodes could be added later
  lifi_nodesContainer.Create (totallifinodes);//we add 1 to the number of nodes as the first node will be LiFi and rest LiFi users
  server_nodesContainer.Create(m_lifi_stations);//We user one server per user device, although we can have different configuration
  ReadUsersFromCSV(totallifinodes);
}

void LiFiController::CreateController ()
{
  m_Centralized_Controller = CreateObject<CentralizedController>();
  m_Centralized_Controller->SetCCNode(m_wifinodesContainer.Get(0));
}

void LiFiController::SetupMobility ()
{
  MobilityHelper mobility;
  Ptr <ListPositionAllocator > m_listPosition1 = CreateObject<
  ListPositionAllocator>();
  std::vector <double> pos;
  pos = m_position[0];
  m_listPosition1->Add(Vector(pos[0]+3, pos[1]+2, pos[2]+4));
  mobility.SetPositionAllocator(m_listPosition1);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (m_wifinodesContainer.Get(0));
  if (m_wifinodesContainer.GetN() > 0)
  {
      mobility.Install (m_wifinodesContainer.Get(0)); 
  }

  Ptr<LiFiNetDevice> apNetDevice = ns3::DynamicCast < LiFiNetDevice>  (lifi_nodesContainer.Get(0)->GetDevice(0));
  std::vector<double> apPosition = m_position[0];
  Vector txPosition = Vector(apPosition[0], apPosition [1], apPosition[2]);
  std::cout<<"apPosition[0], apPosition [1], apPosition[2] "<<apPosition[0]<<" "<<apPosition [1]<<" "<<apPosition[2]<<std::endl;
  mobility.SetMobilityModel ("ns3::LiFiMobilityModel");
  mobility.Install(lifi_nodesContainer.Get(0));
  Ptr<ns3::LiFiMobilityModel>  lifiApMobilityModel  = ns3::DynamicCast < ns3::LiFiMobilityModel>  (lifi_nodesContainer.Get(0)->GetObject<MobilityModel>());
  lifiApMobilityModel->SetPosition(txPosition); 
      
  for (uint16_t j = 1; j < lifi_nodesContainer.GetN(); j++)
  {   
      std::vector<double> rxPositionVector = m_position [j];
      std::cout<<"std::vector<double> rxPositionVector = m_position [j]; "<<rxPositionVector[0]<<" "<<rxPositionVector[1]<<" "<<rxPositionVector[2]<<std::endl;
      if(j==1223 || j ==2322) 
      {
          mobility.SetMobilityModel ("ns3::LiFiConstantVelocityMobilityModel");
          mobility.Install(NodeContainer(lifi_nodesContainer.Get(j)));
          Ptr<ns3::LiFiConstantVelocityMobilityModel>  lifiConstantVelModel  = ns3::DynamicCast < ns3::LiFiConstantVelocityMobilityModel>  (lifi_nodesContainer.Get(j)->GetObject<MobilityModel>());
          Vector v = Vector(0.02,0.04,0.8);
          lifiConstantVelModel->SetPosition(Vector(rxPositionVector[0],rxPositionVector[1],rxPositionVector[2]));
          lifiConstantVelModel->SetVelocity(v); 

      } 
      else if(j >=1 && j<=4)
      {
          ObjectFactory pos;
          pos.SetTypeId ("ns3::GridPositionAllocator");
          pos.Set ("GridWidth", UintegerValue (50));
          pos.Set ("MinX", DoubleValue (1.3));
          pos.Set ("MinY", DoubleValue (0.8));
          pos.Set ("DeltaX", DoubleValue (0.8));
          pos.Set ("DeltaY", DoubleValue (-1.2));
          pos.Set ("LayoutType", EnumValue (GridPositionAllocator::ROW_FIRST));
          Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();                
          mobility.SetMobilityModel ("ns3::LiFiRandomWaypointMobilityModel",
          "Pause", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"),
          "PositionAllocator", PointerValue (taPositionAlloc));
          mobility.Install(NodeContainer(lifi_nodesContainer.Get(j)));
          ns3::Ptr<LiFiRandomWaypointMobilityModel> lifiRandomWaypointMobilityModel =ns3::DynamicCast < ns3::LiFiRandomWaypointMobilityModel>  (lifi_nodesContainer.Get(j)->GetObject<MobilityModel>());

          lifiRandomWaypointMobilityModel->SetPosition(Vector(rxPositionVector[0],rxPositionVector[1],rxPositionVector[2]));
          
        }
      else 
      {
          mobility.SetMobilityModel ("ns3::LiFiMobilityModel");
          mobility.Install(lifi_nodesContainer.Get(j));
          Ptr<ns3::LiFiMobilityModel>  lifiMobilityModel  = ns3::DynamicCast < ns3::LiFiMobilityModel>  (lifi_nodesContainer.Get(j)->GetObject<MobilityModel>());
          lifiMobilityModel->SetPosition(Vector(rxPositionVector[0], rxPositionVector [1], rxPositionVector[2]));
          //lificonstantMobilityNodes.Add(nodeContainer.Get(j));

      }
    }
}

void LiFiController::ConfigureServers ()
{
    //CSMA Helpers
    CsmaHelper csmaHelper;
    csmaHelper.SetChannelAttribute ("DataRate", StringValue ("400Mbps"));
    csmaHelper.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (5)));
    csmaHelper.SetDeviceAttribute ("EncapsulationMode", StringValue ("Llc"));
    csmaHelper.SetDeviceAttribute ("Mtu", UintegerValue (1500));
    NodeContainer cont;
    cont.Add(m_wifinodesContainer.Get(0));
    cont.Add(server_nodesContainer);
    csmaServerNetDevices = csmaHelper.Install (cont);
}

void LiFiController::ConfigureLiFiNetwork ()
{
    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("400Mbps"));
    pointToPoint.SetChannelAttribute ("Delay", StringValue ("30us"));

    NetDeviceContainer p2pDevices;
    p2pDevices = pointToPoint.Install (NodeContainer(m_wifinodesContainer,lifi_nodesContainer.Get(0)));
    Ptr<PointToPointNetDevice> controllerInterfaceNetDevice = ns3::DynamicCast<PointToPointNetDevice> (p2pDevices.Get(0));
    m_Centralized_Controller->SetLiFiAPsNetDeviceInterface(controllerInterfaceNetDevice);//Register Device


    lifiToCCNetDeviceContainer.Add(p2pDevices);
    struct LifiPhyParameters lifiPhyParameters;
    lifiPhyParameters.semiAngle = 60;
    lifiPhyParameters.photoDetectorArea = 1.0e-4;
    lifiPhyParameters.filterGain = 1.0;
    lifiPhyParameters.FOVAngle = 90.0;
    lifiPhyParameters.refractiveIndex = 1.5;
    lifiPhyParameters.dataRateInMBPS = 5.0;
    lifiPhyParameters.beta = 1.0;
    lifiPhyParameters.isVPPM = false;
    lifiPhyParameters.modulation = LiFiErrorModel::OOK;
 
    //Create Phy and MAC Layers for LiFi AP Node
    Ptr<Node> lifiApNode = lifi_nodesContainer.Get(0);
    ns3::Ptr<ns3::LiFiPhyHelper> phyHelper = CreateObject <ns3::LiFiPhyHelper> ();
    phyHelper->Create(lifi_nodesContainer,lifiPhyParameters); 

    ns3::Ptr<ns3::LiFiChannelHelper> chHelper = CreateObject <ns3::LiFiChannelHelper> ();
    chHelper->Create(lifiApNode);

    Ptr<LiFiPhy> aplifiPhy = phyHelper->GetPhy(lifiApNode->GetId());
    aplifiPhy->SetDeviceType("AP");
    chHelper->AddPhy(lifiApNode,aplifiPhy);

    Ptr<LiFiMacHelper> lifiMacHelper = CreateObject <ns3::LiFiMacHelper> (); //::CreateAPMac (Ptr<Node> node, ns3::Ptr<LiFiPhy> lifiPhy)
    NetDeviceContainer apNetdevices = lifiMacHelper->CreateAPMac(lifiApNode,phyHelper->GetPhy(lifiApNode->GetId()),chHelper,p2pDevices.Get(0));    
    lifiNetDevicesContainer.Add(apNetdevices); 

    //Create Phy and Mac Layers for User Nodes
    NodeContainer lifiUserNodes;
    NetDeviceContainer userNetdevices;
    for (uint32_t j = 1; j < (lifi_nodesContainer.GetN()); j++)
    {
        lifiUserNodes.Add(lifi_nodesContainer.Get(j));
        Ptr<LiFiPhy> lifiPhy = phyHelper->GetPhy(lifi_nodesContainer.Get(j)->GetId());
        lifiPhy->SetDeviceType("UD");
        Ptr<LiFiChannel> LiFiChannel= chHelper->GetChannel(lifiApNode->GetId());
        lifiPhy->SetChannel(LiFiChannel);
        userNetdevices.Add(lifiMacHelper->CreateUserDeviceMac(lifi_nodesContainer.Get(j), lifiPhy));
    }  
    lifiNetDevicesContainer.Add(userNetdevices);
    Ptr<LiFiHelper> lifiHelper = CreateObject <LiFiHelper>();
    lifiHelper->AssociateUserDevicesWithAP(lifiUserNodes,lifiApNode,chHelper,phyHelper,lifiMacHelper);
}

void LiFiController::ConfigureWiFiNetwork ()
{
    std::string apManager = "ns3::MinstrelHtWifiManager";
    uint32_t rtsThreshold = 65535;
    double maxPower = 59;
    double minPower = 50;
    uint32_t powerLevels = 10;
    Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (80));
    Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("5000"));
    Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("16000"));
    std::string phyRate = "VhtMcs6";                    /* Physical layer bitrate. */
    
    /* Set up Legacy Channel */
    YansWifiChannelHelper channel;
    //Ptr<YansWifiChannel> wifiChannel = 0;
    channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5e9));
    /* Setup Physical Layer */       
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
    phy.Set ("ShortGuardEnabled", BooleanValue (1));
    // Set MIMO capabilities
    uint8_t nStreams = 1 + (9 / 8); //number of MIMO streams
    phy.Set ("Antennas", UintegerValue (nStreams));
    phy.Set ("MaxSupportedTxSpatialStreams", UintegerValue (nStreams));
    phy.Set ("MaxSupportedRxSpatialStreams", UintegerValue (nStreams));
    phy.SetChannel (channel.Create ());
    // phy.Set ("TxPowerLevels", UintegerValue (1));
    phy.Set ("TxGain", DoubleValue (0));
    phy.Set ("RxGain", DoubleValue (0));
    phy.Set ("RxNoiseFigure", DoubleValue (10));
    phy.Set ("EnergyDetectionThreshold", DoubleValue (-79 + 3));
    phy.SetErrorRateModel ("ns3::YansErrorRateModel");        
    Ssid ssid = Ssid ("IEEE802ac");
    
    WifiMacHelper mac;
   
    WifiHelper wifi;
    wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
    wifi.SetRemoteStationManager (apManager, "RtsCtsThreshold", UintegerValue (rtsThreshold));
    phy.Set ("TxPowerStart", DoubleValue (maxPower));
    phy.Set ("TxPowerEnd", DoubleValue (maxPower));
    mac.SetType ("ns3::StaWifiMac", "ActiveProbing", BooleanValue (false),"Ssid", SsidValue (ssid));
    NodeContainer userNodes;
    for (size_t i = 1; i < lifi_nodesContainer.GetN(); i++)
    {
      userNodes.Add(lifi_nodesContainer.Get(i));
    }
    
    NetDeviceContainer staDevices = wifi.Install (phy, mac, userNodes);
    wifi.SetRemoteStationManager (apManager, "RtsCtsThreshold", UintegerValue (rtsThreshold));
    phy.Set ("TxPowerStart", DoubleValue (minPower));
    phy.Set ("TxPowerEnd", DoubleValue (maxPower));
    phy.Set ("TxPowerLevels", UintegerValue (powerLevels));
    mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
    NetDeviceContainer appDevices = wifi.Install (phy, mac, m_wifinodesContainer.Get(0));
    wifiNetDevices.Add(appDevices.Get(0));
    wifiNetDevices.Add(staDevices);
}

void LiFiController::ReadUsersFromCSV (uint16_t num_nodes)
{
    std::vector <std::string> filesStringVect;
    filesStringVect.push_back("NetworkPositions/lifi-ap1.txt");
    //open pointers to files containing positions for different access points..
    //We first store the file pointers in a vector and then iterate over 
    std::vector <std::fstream*> filesStreamVector;
    for (uint32_t i = 0; i < filesStringVect.size();i++ )
    {
         std::fstream* myFile  = new std::fstream(filesStringVect[i].c_str());
         filesStreamVector.push_back(myFile);
    }

    //Parse Positions from pointers to positions vector
    for (uint32_t i = 0; i < filesStreamVector.size();++i)
    {
        std::string line;
        std::fstream* myFile = (filesStreamVector [i]);
        
        uint32_t posIndex = 0;
        while ( getline(*myFile , line ) )
        {
          if (posIndex > num_nodes) break;
          
          std::istringstream is( line );  
          m_position.push_back(std::vector<double>( std::istream_iterator<double>(is),std::istream_iterator<double>() ) );
          posIndex ++;
        }
 	      myFile->close();
        delete myFile;
     }
}

void LiFiController::InstallInternetStack ()
{
  InternetStackHelper stack;
  stack.InstallAll ();
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.0.0", "255.255.255.0"); 
  //Server Nodes ----> wifi node
  serversIpv4interfaceContainer = ipv4.Assign(csmaServerNetDevices);
  //configure default route in server nodes
  for (uint32_t z = 1; z < serversIpv4interfaceContainer.GetN();z++)
  {
    Ptr < Ipv4StaticRouting > staticRouting=  ipv4RoutingHelper.GetStaticRouting(csmaServerNetDevices.Get(z)->GetNode()->GetObject<Ipv4>());
    staticRouting->SetDefaultRoute(serversIpv4interfaceContainer.GetAddress(0,0), 1,1); //This block is for sending from WIFI AP to the MT Ipv4Address
  }

  ipv4.SetBase("10.1.1.0", "255.255.255.0"); //Set up WiFi Network 
  wifiIpv4interfaceContainer = ipv4.Assign(wifiNetDevices);
  //Configure IP address LiFi AP with LiFi user device
  ipv4.SetBase("10.1.2.0", "255.255.255.0"); //Set up WiFi Network 
  lifiAPtoCCInterfaceContainer = ipv4.Assign(lifiToCCNetDeviceContainer);
  Ptr<Ipv4> wifiApIpObject              =  m_wifinodesContainer.Get(0)->GetObject<Ipv4> (); //ipv4A
  int32_t interfaceIndex                  = wifiApIpObject->GetInterfaceForDevice (lifiToCCNetDeviceContainer.Get(0));
  Ptr < Ipv4StaticRouting > staticRoutingWifiAP   = ipv4RoutingHelper.GetStaticRouting(m_wifinodesContainer.Get(0)->GetObject<Ipv4>());
  lifiAPIpv4interfaceContainer = ipv4.Assign(lifiNetDevicesContainer);//WiFi AP Node -----> LiFi AP assigned IP addresses container
  for (uint32_t z = 1; z < lifiNetDevicesContainer.GetN();z++)
  {
      staticRoutingWifiAP->AddHostRouteTo (lifiAPIpv4interfaceContainer.GetAddress(z,0),lifiAPtoCCInterfaceContainer.GetAddress(1,0),interfaceIndex,1);

    //Ptr < Ipv4StaticRouting > staticRoutingHandleUserDevice =  ipv4RoutingHelper.GetStaticRouting(lifiNetDevicesContainer.Get(z)->GetNode()->GetObject<Ipv4>());
   // staticRoutingHandleUserDevice->SetDefaultRoute(lifiAPIpv4interfaceContainer.GetAddress(0,0),2,1); //This block is for sending from WIFI AP to the MT Ipv4Address
  }
}

void LiFiController::InstallLiFiNodesApplications (std::string dataRate)
{  
  uint32_t payloadSize = 1448;  
  const double min = 0.1;
  const double max = 0.2;
  Ptr<UniformRandomVariable> uniform = CreateObject<UniformRandomVariable> ();
  uniform->SetAttribute ("Min", DoubleValue (min));
  uniform->SetAttribute ("Max", DoubleValue (max));
  std::string protocolTypeStr = "ns3::UdpSocketFactory";
  uint16_t server_port = 900;
  PacketSinkHelper staSinkAppHelper (protocolTypeStr, Address (InetSocketAddress (Ipv4Address::GetAny (), server_port)));
  ApplicationContainer lifiSinkApps;
  for (size_t i = 1; i < lifi_nodesContainer.GetN(); i++)
  {
      ApplicationContainer apps =staSinkAppHelper.Install (lifi_nodesContainer.Get(i));
      apps.Start (Seconds (uniform->GetValue()));
      lifiSinkApps.Add(apps);
  }

  std::ostringstream stream;
  stream<<server_port;
  std::string portStr = stream.str();
  lifiSinkApps.Stop (Seconds (simulationTime+1));
  std::unordered_map<uint32_t, Ptr<UserDeviceDataModel> > userDataModelMap;
  std::unordered_map <std::string, std::unordered_map<uint32_t, Ptr<UserDeviceDataModel> > > apUserDataModelMaps;

  for (uint32_t i = 0; i < server_nodesContainer.GetN(); i++)
  {
      Ptr <Node> node                        = lifiSinkApps.Get(i)->GetNode();
      Ptr<Ipv4> ipv4A                        = node->GetObject<Ipv4> ();
      Ipv4Address ipv4                       = ipv4A->GetAddress(2,0).GetLocal();
      OnOffHelper sourceAppHelper (protocolTypeStr, (InetSocketAddress (ipv4,server_port)));
      sourceAppHelper.SetAttribute ("PacketSize", UintegerValue (payloadSize));
      sourceAppHelper.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));//onNormRandVariable"));//StringValue ("ns3::ExponentialRandomVariable[Mean=0.8]"));
      sourceAppHelper.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));//offNormRandVariable);_//StringValue ("ns3::ExponentialRandomVariable[Mean=0.5]"));
      sourceAppHelper.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));

      ApplicationContainer serverApps = sourceAppHelper.Install (server_nodesContainer.Get(i));
      double strtval = uniform->GetValue();
      serverApps.Start (Seconds (strtval+0.3));
      serverApps.Stop (Seconds (simulationTime+0.5));

      Ptr <Node> node1                        = server_nodesContainer.Get(i);
      Ptr<Ipv4> ipv4A1                        = node1->GetObject<Ipv4> ();
      Ptr<LiFiNetDevice> udNetDev = DynamicCast<LiFiNetDevice> (node->GetDevice(0));

      Ptr<LiFiMacLow> macLow = udNetDev->GetMac()->GetMacLow();
      Ptr<LiFiPhy>  phy=  macLow->GetPhy ();
      //m_phyVector.push_back(phy);
      ns3::Ptr<UserDeviceDataModel > userDeviceDataModel = CreateObject<UserDeviceDataModel>();
      userDeviceDataModel->SetUDPhy(phy);
      userDeviceDataModel->SetDestinationAddress (ipv4);
      userDeviceDataModel->SetSinkApplication (DynamicCast<PacketSink>(lifiSinkApps.Get(i)));
      userDeviceDataModel->SetSourceApplication(DynamicCast<ns3::OnOffApplication>(serverApps.Get(0)));
      userDeviceDataModel->SetDestinationPort(server_port);
      userDeviceDataModel->AddLiFiAPNode(lifi_nodesContainer.Get(0));
      userDeviceDataModel->SetCentralizedController(m_Centralized_Controller);
      userDeviceDataModel->SetTimeToTrigger(200);
      userDeviceDataModel->SetLagValue(std::make_pair<>(30,10));
      userDeviceDataModel->SetHandoverDelay(50);
      userDataModelMap.insert(std::make_pair(node->GetId(), userDeviceDataModel));
      m_Centralized_Controller->RegisterUsreDevice(node->GetId(),userDeviceDataModel);
    }
    //apUserDataModelMaps.insert(std::make_pair(portStr, userDataModelMap));
    //m_Centralized_Controller->SetUserDataModelMaps(apUserDataModelMaps);
}




