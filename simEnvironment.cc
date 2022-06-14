

#include "ns3/core-module.h"
#include "ns3/opengym-module.h"
#include "ns3/gnuplot.h"
#include "ns3/command-line.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-address.h"
#include "ns3/on-off-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/olsr-helper.h"

using namespace ns3;

//Parametros de simulación (Distancia, Tamaño por paquete, paquetes)

double distance = 30;  
uint32_t packetSize = 1000; 
uint32_t numPackets = 100;

uint32_t nodesNumber = 15;
uint32_t receivedPackets = 0;

NodeContainer nodes;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhocGrid");

void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
    { 
      receivedPackets = receivedPackets + 1;
      //NS_LOG_UNCOND ("1 paquete");
    }
}

static void GenerateTraffic (
Ptr<Socket> socket, 
uint32_t packetSize, 
uint32_t packets, 
Time packetTime 
){
  if (packets > 0)
    {
      socket->Send (Create<Packet> (packets));
      Simulator::Schedule (
	packetTime, 
	&GenerateTraffic,
        socket, 
	packetSize,
	packets - 1, 
	packetTime);
    }
  else
    {
      socket->Close ();
    }
}


//-----------------Funciones de simulacción en relación con el agente------------

// Definición entono percibido por agente

Ptr<OpenGymSpace> GetObservationSpace(){
  std::vector <uint32_t> shape {nodesNumber,};
  std::string type = TypeNameGet<uint32_t>();


  Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace> (0,20,shape,type);
  NS_LOG_UNCOND("Espacio de percepción: "<<space);
  return space;
}

Ptr<OpenGymDataContainer> GetObservation(){
  std::vector<uint32_t> shape = {nodesNumber,};
  Ptr<OpenGymBoxContainer<uint32_t>> box =  CreateObject<OpenGymBoxContainer<uint32_t>> (shape);

  Ptr<UniformRandomVariable> rndVariable = CreateObject<UniformRandomVariable>();

  int randomNumber;
  Vector position;
  uint32_t min = 0;
  uint32_t max = distance;

  //Cambiar posición del nodo

  for (uint32_t i = 0; i < nodesNumber; i++){
    
    randomNumber = rndVariable->GetInteger(min,max);
    box->AddValue(randomNumber);

    //Posición aleatoria del nodo en x

    position =  Vector(randomNumber,0,0);

    Ptr<MobilityModel> mobility = nodes.Get(i)->GetObject<MobilityModel>();
    mobility->SetPosition(position);
  }

/*
 Vector position = Vector(0,0,0);
  double px = distance;
  for (int i = 0; i < 15; i++){
    Ptr<MobilityModel> mobility = nodes.Get(i)->GetObject<MobilityModel>();
    position = mobility->GetPosition(); 
    px = px - 1;
    mobility->SetPosition(Vector(px,0,0));
    position = mobility->GetPosition();
    NS_LOG_UNCOND("POS "<<position.x); 
  }
*/
  NS_LOG_UNCOND ("Percepción: " << box);
  return box;
}

// Determinar acciones a realizar

Ptr<OpenGymSpace> GetActionSpace(){
  Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace> (nodesNumber);
  NS_LOG_UNCOND ("Espació de acción: " << space);
  return space;
}

//Acciones

bool Actions(Ptr<OpenGymDataContainer> action){
  Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
  NS_LOG_UNCOND ("Acción: " << action);
  return true;
}

void ScheduleNextStateRead(double stepTime, Ptr<OpenGymInterface> openGym)
{
  Simulator::Schedule (Seconds(stepTime), &ScheduleNextStateRead, stepTime, openGym);
  openGym->NotifyCurrentState();
}

//Recompensa: medida en el numero de paquetes que fueron recibidos

float Reward(){
  NS_LOG_UNCOND ("Refuerzo / Premio: " << receivedPackets);
  return receivedPackets;
}

bool EndAgent()
{

  bool isGameOver = false;
  bool test = false;
  static float stepCounter = 0.0;
  stepCounter += 1;
  if (stepCounter == 20 && test) {
      isGameOver = true;
  }
  return isGameOver;
}


int main (int argc, char *argv[]){

  std::string phyMode ("DsssRate2Mbps");

  //Nodos de destino y fuente 

  uint32_t receiveN = 0;
  uint32_t sourceN = 14;

  bool tracing = true;

  //Tiempos de simulación (Seg.)

  double simulationTime = 20.0;
  double startTraffic = 10.0; 
  double step = 1.0;

  //Configuración del entorno

  uint32_t envGatewayPort = 5555;
 
  //Configuración de opengym

  CommandLine cmd;
  cmd.AddValue("distance","Distancia",distance);
  cmd.AddValue("packetSize","Tamaño por paquete",packetSize);
  cmd.AddValue("openGymPort","Puerto",envGatewayPort);
  cmd.AddValue("rNode","Nodo receptor",receiveN);
  cmd.AddValue("sNode","Nodo fuente",sourceN);
  cmd.AddValue("simTime","Duración de la simulación",simulationTime);
  cmd.AddValue("stTraffic","Inicio de envió de paquetes",startTraffic);
  cmd.Parse(argc, argv);
  
  Ptr<OpenGymInterface> openGym = CreateObject<OpenGymInterface> (envGatewayPort);
  openGym->SetGetObservationSpaceCb( MakeCallback (&GetObservationSpace) );
  openGym->SetGetObservationCb( MakeCallback (&GetObservation) );
  openGym->SetGetActionSpaceCb( MakeCallback (&GetActionSpace) );
  openGym->SetExecuteActionsCb( MakeCallback (&Actions) );
  openGym->SetGetRewardCb( MakeCallback (&Reward) );
  openGym->SetGetGameOverCb( MakeCallback (&EndAgent) );

  //Tempo entre paquetes
  
  Time interPacketInterval = Seconds (step);

  
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));

//Crear contenedor de nodos

  nodes.Create (nodesNumber);

//Configuración red inalambrica

  WifiHelper wifi;
  YansWifiPhyHelper wifiPhy;
  //YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  
  wifiPhy.Set ("RxGain", DoubleValue (-10) );
  
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

//Protocolo de red 80211b

  WifiMacHelper wifiMac;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

//Configuración a modo adhoc

  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (distance),
                                 "DeltaY", DoubleValue (distance),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);


  OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (olsr, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list); 
  internet.Install (nodes);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

//Configuranción nodo origen y llegada

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (nodes.Get (receiveN), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (nodes.Get (sourceN), tid);
  InetSocketAddress remote = InetSocketAddress (i.GetAddress (receiveN, 0), 80);
  source->Connect (remote);

//Trazado de rutas

  if (tracing == true)
    {

      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("wifi-simple-adhoc-grid.tr"));
      wifiPhy.EnablePcap ("wifi-simple-adhoc-grid", devices);
      
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.routes", std::ios::out);
      olsr.PrintRoutingTableAllEvery (Seconds (2), routingStream);
      Ptr<OutputStreamWrapper> neighborStream = Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.neighbors", std::ios::out);
      olsr.PrintNeighborCacheAllEvery (Seconds (2), neighborStream);

    }

//Primera generciòn de trafico a partir del segundo 10

  Simulator::Schedule (Seconds (startTraffic), &GenerateTraffic,
                       source, packetSize, numPackets, interPacketInterval);
  NS_LOG_UNCOND ("Distancia"<<distance);
  NS_LOG_UNCOND ("Tamaño por paquete"<<packetSize);
  NS_LOG_UNCOND ("Numero de paquetes"<<numPackets);

//Realizar primera observación del entorno a partir del segundo 1

  Simulator::Schedule (Seconds(1.0), &ScheduleNextStateRead, step, openGym);

  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run ();

  openGym->NotifySimulationEnd();

  Simulator::Destroy ();

  return 0;
}
