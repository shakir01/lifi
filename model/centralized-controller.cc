
#include "centralized-controller.h"
#include "ns3/lifi-ap-mac.h"
#include "ns3/assert.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/abort.h"
#include "ns3/wifi-ap-net-device.h"
namespace ns3
{

UserDeviceDataModel::UserDeviceDataModel ()
{
    this->rxBytesVector.push_back(0);
    m_lastTotalRxBytes = 0;
    isUserMobile = false;
    isHandoverInProgress = false;
    m_time_to_trigger = 0;
    m_handoverDelay   = 0;
    m_isHandoverCompleted = false;
}

void UserDeviceDataModel::SetUserId (uint32_t id)
{
    this->u_id = id;
}

uint32_t  UserDeviceDataModel::GetUserId(void)
{
    return u_id;
}

void UserDeviceDataModel::AddSimulationTime (double simTime)
{
    simTimeVect.push_back(simTime);
}

void UserDeviceDataModel::SetForwardLagValue (uint32_t frdLag)
{
    m_lagAfterTTT = frdLag;
}

uint32_t UserDeviceDataModel::GetForwardLagValue ()
{
    return m_lagAfterTTT;
}

std::vector <double>  UserDeviceDataModel::GetHistoricalSimTime(void)
{
    return simTimeVect;
}

void UserDeviceDataModel::AddSNR(double snr, double simTime)
{
    snrVect.push_back (snr);
}

std::vector <double>  UserDeviceDataModel::GetHistoricalSNR(void)
{
    return snrVect;
}

 void UserDeviceDataModel::AddLiFiAPNode(Ptr<Node> node)
 {
   m_lifiApNodesVector.push_back(node);
 }

Ptr<Node> UserDeviceDataModel::GetLiFiAPNode()
{
    return m_lifiApNodesVector[m_lifiApNodesVector.size()-1];

}

void UserDeviceDataModel::SetHandoverDelay(double db)
{
    m_handoverDelay = db;
}

double UserDeviceDataModel::GetHandoverDelay()
{
    return m_handoverDelay;
}

void UserDeviceDataModel::AddErrorRate(double errorRate, double simTime)
{
    errRateVect.push_back (errorRate);
}

std::vector <double>  UserDeviceDataModel::GetHistoricalErrorRate(void)
{
    return errRateVect;
}

void UserDeviceDataModel::AddPathLoss(double loss, double simTime)
{
    lossVect.push_back(loss);
}

std::vector <double>  UserDeviceDataModel::GetHistoricalPathLoss(void)
{
    return lossVect;
}

void UserDeviceDataModel::AddInstantThroughput(double instantThroughput, double simTime)
{
    throughputVect.push_back(instantThroughput);
}

std::vector <double>  UserDeviceDataModel::GetHistoricalInstantThroughput(void)
{
    return throughputVect;
}

void UserDeviceDataModel::AddLastRxBytes (uint64_t lastRxBytes)
{
  rxBytesVector.push_back(lastRxBytes);
}

void UserDeviceDataModel::SetLastTotalRxBytes (void)
{
    rxBytesVector.push_back(this->GetSinkApplication()->GetTotalRx ());
}

std::vector <uint64_t>  UserDeviceDataModel::GetHistoricalRxBytes (void)
{
    return rxBytesVector;
}

uint64_t  UserDeviceDataModel::GetLastRxBytes (void)
{
    return rxBytesVector[rxBytesVector.size() - 1];
}

void UserDeviceDataModel::SetSinkApplication (ns3::Ptr<PacketSink> app)
{
    m_sinkApplication = app;
}

ns3::Ptr<PacketSink> UserDeviceDataModel::GetSinkApplication (void)
{
    return m_sinkApplication;
}

void UserDeviceDataModel::SetSourceApplication (ns3::Ptr<OnOffApplication> onOffApplication)
{
    m_OnOffApplication = onOffApplication;
}

ns3::Ptr<OnOffApplication> UserDeviceDataModel::GetSourceApplication (void)
{
    return m_OnOffApplication;
}

void UserDeviceDataModel::SetDestinationAddress (Ipv4Address destination)
{
    m_destinationAddress =  destination;
}

ns3::Ipv4Address UserDeviceDataModel::GetDestinationAddress (void)
{
    return m_destinationAddress;
}


void UserDeviceDataModel::SetSourcePort(uint16_t port)
{
    source_port = port;
}

uint16_t UserDeviceDataModel::GetSourcePort()
{
    return  source_port;
}

void UserDeviceDataModel::SetDestinationPort(uint16_t port)
{
    destination_port = port;
}

uint16_t UserDeviceDataModel::GetDestinationPort()
{
    return  destination_port ;
}

bool UserDeviceDataModel::IsHandoverCriteriaSatisfied(uint32_t reprots_count)
{
  std::vector<double> snrVect = GetHistoricalSNR();
  //uint32_t id = this->GetSinkApplication()->GetNode()->GetId();
  double sum = 0;
  if (snrVect.size() < 41)
  {
    return false;
  }

// std::cout<<"IsHandoverRequiredForDevice(std::vector<double> vec, uint32_t id) Time "<<Simulator::Now().GetSeconds()<<" node "<<id<<std::endl;
  for (size_t i = snrVect.size()-1; i >= snrVect.size()-reprots_count; i--)
  {
    //  std::cout<<(10*log(vec[i]))<<"  ";
     // if (vec[i] <= 0.1)
   //   {
        sum += snrVect[i];
    //  }
    //  else
  }

  //std::cout<<"\naverage "<<(sum/15.0)<<"\n\n"<<std::endl;
  if ((sum/reprots_count) <= 20.50)
  {
      //std::cout<<"\naverage "<<(sum/reprots_count)<<"\n\n"<<std::endl;
    return true;
  }

  return false;
}

bool UserDeviceDataModel::IsHandoverHandoverRequired ()
{
    if (m_isHandoverCompleted)
    {
        return false;
    }
    
    if (!isHandoverInProgress && IsHandoverCriteriaSatisfied(m_lagPair.first))
    {
        return true;
    }

    return false;
}

void UserDeviceDataModel::SetUDPhy (Ptr<LiFiPhy> phy)
{
    m_lifiphy = phy;
}

Ptr<LiFiPhy> UserDeviceDataModel::GetUdPhy ()
{
    return m_lifiphy;
}

void UserDeviceDataModel::SetLagValue (std::pair<uint32_t,uint32_t> lag)
{
    m_lagPair = lag;
}

std::pair<uint32_t,uint32_t> UserDeviceDataModel::GetLagValue ()
{
    return m_lagPair;
}

double UserDeviceDataModel::GetTimeToTrigger ()
{
    return m_time_to_trigger;
}

void UserDeviceDataModel::SetSchedularCheckTime(double db)
{
    m_kpi_monetoringTime = db;
}

double UserDeviceDataModel::GetSchedularCheckTime()
{
   return m_kpi_monetoringTime;
}

void UserDeviceDataModel::SetTimeToTrigger (double ttt)
{
    m_time_to_trigger = ttt;
    //m_lifiphy->SetHandoverTriggered(m_time_to_trigger,m_lag);
}

void UserDeviceDataModel::InitiateHandover()
{
    isHandoverInProgress = true;
    Simulator::Schedule (MilliSeconds (m_time_to_trigger), &UserDeviceDataModel::CheckHandoverMetricsAfterTTTExpirey,this);
}

void UserDeviceDataModel::SetCentralizedController (Ptr<CentralizedController> centralizedController)
{
    m_CentralizedController = centralizedController;
}

void UserDeviceDataModel::CheckHandoverMetricsAfterTTTExpirey ()
{
    if (IsHandoverCriteriaSatisfied((m_lagPair.second)))
    {
        Simulator::Schedule (MilliSeconds (m_handoverDelay), &UserDeviceDataModel::CompleteHandover,this);
    }
    else
    {
        m_time_to_trigger = 0;
        isHandoverInProgress = false;
    }
}

void UserDeviceDataModel::CompleteHandover()
{
    isHandoverInProgress = false;
    m_isHandoverCompleted = true;
    m_CentralizedController->PerformVerticalHandover(this);
}

double UserDeviceDataModel::CalculateInstantThroughput (double time)
{
    uint64_t lastRx = this->GetLastRxBytes();
    uint64_t currentRxBytes = this->GetSinkApplication()->GetTotalRx ();
    double cur = ((currentRxBytes- lastRx) * (double) 8.0) / (1e3);     /* Convert Application RX Packets to MBits. */
    this->AddLastRxBytes(currentRxBytes);
    AddInstantThroughput(cur, time);
    return cur;
 }

 void UserDeviceDataModel::ResetDataStructures ()
{
    this->snrVect.clear();
    this->errRateVect.clear();
    this->lossVect.clear();
    this->throughputVect.clear();
 }



CentralizedController::CentralizedController ()
{
    m_isHandoverEnabled = true;
    m_isFirstIteration  = true;
    Simulator::Schedule (Seconds (1), &CentralizedController::KPI_Monitoring,this);


}

CentralizedController::~CentralizedController ()
{

}

void CentralizedController::SetCCNode (Ptr<Node> controllernode)
{
    m_controller_node = controllernode;
}

Ptr<Node> CentralizedController::GetCCNode(void)
{
    return m_controller_node;
}

Ptr<UserDeviceDataModel> CentralizedController::FindUserWithID(uint32_t userId)
{
  // Do stuff
  std::unordered_map <uint32_t, Ptr<UserDeviceDataModel> >::  iterator itr1;
  itr1 = m_userDataModelMaps.find(userId);
  if (itr1 != m_userDataModelMaps.end())
  {
          Ptr<UserDeviceDataModel> userDeviceDataModel = itr1->second;
          return userDeviceDataModel;
  }

  return 0;
}

void CentralizedController::RegisterUsreDevice(uint32_t devid, Ptr<UserDeviceDataModel> userDeviceDataModel)
{
    m_userDataModelMaps.insert(std::make_pair(devid, userDeviceDataModel));
}


void CentralizedController::SetLiFiAPsNetDeviceInterface(Ptr<ns3::PointToPointNetDevice> netdevice)
{
    netdevice->SetReceiveCallback(MakeCallback (&CentralizedController::LiFiAPMeasurementReports, this));
    m_lifiNetDevicesContainer.Add(netdevice);
}

void CentralizedController::SetWiFiAPsNetDeviceInterface(Ptr<ns3::WifiNetDevice> netdevice)
{
    netdevice->SetReceiveCallback(MakeCallback (&CentralizedController::WiFiAPMeasurementReports, this));
    m_wifiNetDevicesContainer.Add(netdevice);
}

bool CentralizedController::LiFiAPMeasurementReports(Ptr<NetDevice> netDevice,Ptr<const Packet> packet,uint16_t protocolId, const Address & address)
{
    m_isHandoverEnabled = true;
    std::stringstream m_CSVfileNamestream;
    if (m_isHandoverEnabled)
    {
        m_CSVfileNamestream<<"LiFi-Network/"<<"Lifi_Stations_Instant_Data_handover_active_apid "<<netDevice->GetNode()->GetId()<<".csv";

    }
    else
    {
        m_CSVfileNamestream<<"LiFi-Network/"<<"Lifi_Instant_Data_no_handover_measurement_time _apid "<<netDevice->GetNode()->GetId()<<".csv";
    }

    std::string cSVfileNamestream = m_CSVfileNamestream.str();
    std::ofstream dat_file_out (cSVfileNamestream.c_str (), std::ios::app);

    if (m_isFirstIteration)
    {
        dat_file_out<<"nodes\t "<<"ID \t " << "Time \t " << " SNR \t "<< " BER\t "<<" Loss\t "<<" Throughput\t "<< std::endl;
        m_isFirstIteration = false;
    }



    double record_time = Simulator::Now().GetSeconds();
    Ptr<Packet> cpPacket = packet->Copy();
    ns3::MeasurementReport measurementReport;
    cpPacket->RemovePacketTag(measurementReport);
    double snr = measurementReport.GetSNR()[0];
    Ptr<UserDeviceDataModel> userDeviceDataModel = FindUserWithID(measurementReport.GetDeviceID());

    userDeviceDataModel->AddSimulationTime(record_time);
    userDeviceDataModel->AddSNR(snr, record_time);

    userDeviceDataModel->AddErrorRate(0,record_time);
    userDeviceDataModel->AddPathLoss(0,record_time);

    dat_file_out << measurementReport.GetDeviceID() <<", \t " << record_time << ", \t "<< snr  << ", \t " << 0<<", \t "<< 0<< ", \t "<<0<< std::endl;

   return true;
 }

bool CentralizedController::WiFiAPMeasurementReports(Ptr<NetDevice> netDevice,Ptr<const Packet> packet,uint16_t protocolId, const Address & address)
{

    return false;
}

Ptr<Ipv4StaticRouting> CentralizedController::GetStaticRouting (Ptr<Ipv4> ipv4) const
 {
   Ptr<Ipv4RoutingProtocol> ipv4rp = ipv4->GetRoutingProtocol ();
   NS_ASSERT_MSG (ipv4rp, "No routing protocol associated with Ipv4");
   if (DynamicCast<Ipv4StaticRouting> (ipv4rp))
   {
       return DynamicCast<Ipv4StaticRouting> (ipv4rp);
   }

   if (DynamicCast<Ipv4ListRouting> (ipv4rp))
     {
       Ptr<Ipv4ListRouting> lrp = DynamicCast<Ipv4ListRouting> (ipv4rp);
       int16_t priority;
       for (uint32_t i = 0; i < lrp->GetNRoutingProtocols ();  i++)
         {
           Ptr<Ipv4RoutingProtocol> temp = lrp->GetRoutingProtocol (i, priority);
           if (DynamicCast<Ipv4StaticRouting> (temp))
             {
               return DynamicCast<Ipv4StaticRouting> (temp);
             }
         }
     }
   return 0;
 }

std::pair<int,uint32_t>  CentralizedController::GetWiFiNetDeviceInterfaceIndex (Ptr<Node> node)
{
    for (uint32_t i = 0; i < node->GetNDevices(); i++)
    {
       Ptr<NetDevice> netDevice = node->GetDevice(i);
       Ptr <WifiNetDevice> wifiNetDevice = DynamicCast<WifiNetDevice>(netDevice);
       if (wifiNetDevice)
       {
           return std::make_pair(1,i);
       }

    }

    return std::make_pair(-1,-1);
}



void CentralizedController::TransferApplicationLiFiToWiFi(Ptr<UserDeviceDataModel> userDeviceDataModel)
{
   //std::cout<<"void CentralizedController::TransferApplicationLiFiToWiFi(Ptr<UserDeviceDataModel> userDeviceDataModel)"<<std::endl;
   Ptr<Node> node = GetCCNode();
   bool isRouteRemoved = false;
   Ptr<Ipv4> ipv4proto = node->GetObject<Ipv4> ();
   Ptr < Ipv4StaticRouting > staticRouting   = GetStaticRouting(ipv4proto);
   Ipv4RoutingTableEntry flagedRouteEntry;

   for (uint32_t i = 0; i < staticRouting->GetNRoutes(); i++)
   {
       Ipv4RoutingTableEntry routeEntry = staticRouting->GetRoute(i);
        //std::cout<<"userDeviceDataModel->GetDestinationAddress() userDeviceDataModel->GetDestinationAddress() "<<userDeviceDataModel->GetDestinationAddress()<<" routeEntry.GetDest () "<<routeEntry.GetDest ()<<std::endl;
       if (routeEntry.GetDest () == userDeviceDataModel->GetDestinationAddress())
       {
           staticRouting->RemoveRoute(i);
           isRouteRemoved = true;
           break;
       }
   }

   if (isRouteRemoved)
   {
        Ptr < Ipv4StaticRouting > staticRouting   = GetStaticRouting(ipv4proto);
        std::pair<int,uint32_t> ind_pair = GetWiFiNetDeviceInterfaceIndex(GetCCNode());
        if (ind_pair.first != -1)
        {
            Ptr<Node> udnode = userDeviceDataModel->GetSinkApplication()->GetNode();
            Ptr<Ipv4> ipv4A1                        = udnode->GetObject<Ipv4> ();//node->GetDevice(2)
            Ptr<WifiNetDevice> txNetDev = ns3::DynamicCast <WifiNetDevice> (udnode->GetDevice(1));
            int32_t interfaceIndex1                = ipv4A1->GetInterfaceForDevice (txNetDev);
            Ipv4InterfaceAddress ipv4Addr = Ipv4InterfaceAddress (userDeviceDataModel->GetDestinationAddress(), Ipv4Mask ("/24"));
            std::cout<<"Time: "<<Simulator::Now().GetSeconds()<<" GetDestinationAddress(), Ipv4Mask (/24): ipv4A1 "<<ipv4A1<<" ipv4Addr "<<ipv4Addr.GetLocal()<<" interfaceIndex1 "<<interfaceIndex1<<" txNetDev "<<txNetDev<<" udnodeid "<<udnode->GetId()<<" index "<<ind_pair.second<<std::endl;
            ipv4A1->AddAddress (interfaceIndex1, ipv4Addr);
            ipv4A1->SetMetric (interfaceIndex1, 1);
            ipv4A1->SetUp (interfaceIndex1);
            Ptr<Ipv4> ccipobject                        = GetCCNode()->GetObject<Ipv4> ();//node->GetDevice(2)
            Ptr<WifiNetDevice> ccwifinetdevice = ns3::DynamicCast <WifiNetDevice> (GetCCNode()->GetDevice(ind_pair.second));
            int32_t wifiipinterface                = ccipobject->GetInterfaceForDevice (ccwifinetdevice);
            staticRouting->AddHostRouteTo (ipv4Addr.GetLocal(),wifiipinterface,1);
            Simulator::Schedule (Seconds (0.9), &CentralizedController::PrintDroppedPackets,this,userDeviceDataModel); 
        }
   }
}

void CentralizedController::PrintDroppedPackets(Ptr<UserDeviceDataModel> userDeviceDataModel)
{
    /*Ptr<Node> apNode = ns3::NodeList::GetNode(userDeviceDataModel->GetCurrentAPID());
    Ptr<LiFiNetDevice> lifitxNetDev = ns3::DynamicCast <LiFiNetDevice> (apNode->GetDevice(1));
    Ptr<LiFiRegularMac> apRegularMac = lifitxNetDev->GetMac();
    Ptr<LiFiTxMac> lifiTxMac = apRegularMac->GetLiFiTxMac();
    Ptr<LiFiMacQueue> macQueue = lifiTxMac->GetMacQueue();
    std::cout<<"Dropped Packets "<<macQueue->GetDroppedPackets()<<std::endl;
    userDeviceDataModel->GetUdPhy()->SetHandoverTriggered(userDeviceDataModel->GetTimeToTrigger(),userDeviceDataModel->GetLagValue(),macQueue->GetDroppedPackets());
    */
    /*Ptr<Node> apNode = ns3::NodeList::GetNode(userDeviceDataModel->GetCurrentAPID());
    Ptr<LiFiNetDevice> lifitxNetDev = ns3::DynamicCast <LiFiNetDevice> (apNode->GetDevice(1));
    Ptr<LiFiRegularMac> apRegularMac = lifitxNetDev->GetMac();
    Ptr<LiFiTxMac> lifiTxMac = apRegularMac->GetLiFiTxMac();
    Ptr<LiFiMacQueue> macQueue = lifiTxMac->GetMacQueue();*/
}

void CentralizedController::KPI_Monitoring()
{
 for (auto& itr: m_userDataModelMaps)
 {  
    Ptr<UserDeviceDataModel> userDeviceDataModel = itr.second;
    if (userDeviceDataModel->IsHandoverHandoverRequired())
    {
        userDeviceDataModel->InitiateHandover();
        //userDeviceDataModel->SetHandoverDelay(0);
        //userDeviceDataModel->SetLagValue(30);
        //userDeviceDataModel->SetForwardLagValue(16);
        //userDeviceDataModel->SetTimeToTrigger(800);
    }
  }
  Simulator::Schedule (MilliSeconds (300), &CentralizedController::KPI_Monitoring,this);
}

void CentralizedController::PerformVerticalHandover (Ptr<UserDeviceDataModel> userDeviceDataModel)
{
    this->TransferApplicationLiFiToWiFi(userDeviceDataModel);
}

}
