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

//Params
double distance = 30;
uint32_t nodesNum = 15;  
uint32_t packetDim = 1000; 
uint32_t packetNum = 100;
uint32_t receivedPackets = 0;

NodeContainer nodes;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhocGrid");

//contidad de paquetes recibidos
void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
    { 
      receivedPackets = receivedPackets + 1;
    }
}

// recibe como parametros:
// -Smart Pointer al socket
// -tamaño del paquete de aplicación enviado
// -numero de paquetes generados
// -intervalo
static void GenerateTraffic (
Ptr<Socket> socket, 
uint32_t packetDim, 
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
	packetDim,
	packets - 1, 
	packetTime);
    }
  else
    {
      socket->Close ();
    }
}

//Funciones de simulacion por el lado desde el agente
// Al ser un ambiente generico solo se necesita instanciar la interfaz de Open-aiGym
// he implementar las funciones propias de esa clase.

//-----------------------------------FUNCIONES--------------------------------------------



// Determina las acciones a realizar, solo se llama durante la inicialización del ambiente
Ptr<OpenGymSpace> GetActionSpace(){
  Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace> (nodesNum);
  NS_LOG_UNCOND ("Action Space -> " << space);
  return space;
}

//Define el espacio de observacion, solo se llama durante la inicialización del ambiente
//el framework de Open Ai acepta los espacios mas usados como:

// Discrete — a single discrete number with value between 0 and N.
// Box — a vector or matrix of numbers of single type (actual)
// with values bounded between low and high limits.
// Tuple — a tuple of simpler spaces.
// Dict — a dictionary of simpler spaces.
Ptr<OpenGymSpace> GetObservationSpace(){
  float low = 0.0;
  float high = 100.0;
  std::vector <uint32_t> shape {nodesNum,};
  std::string type = TypeNameGet<uint32_t>();
  Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace> (low,high,shape,type);
  NS_LOG_UNCOND("Observation Space -> "<< space);
  return space;
}

//Durante cada paso ejecutado, EL framework recolecta el estado actual del ambiente
// llamando las siguientes funciones:

// GetObservation –  Recolecta los valores de las variables o parametros observables
//                   en cualquier nodo de la red en cada capa de la la pila del
//                   protocolo de red.
// GetReward – mide la recompensa alcanzada durante el último paso
// 3) GetGameOver – comprobar una condición de fin de 'juego' predefinida
// 4) GetExtraInfo – obtener una información extra asociada al estado del ambiente


//  Primero, El contenedor Box de datos se crea de acuerdo con la definición del
// espacio de observación. Entonces el contenedor se llena con el tamaño actual de la
//cola de interfaz WiFi de cada nodo.

Ptr<OpenGymDataContainer> GetObservation(){
  std::vector<uint32_t> shape = {nodesNum,};
  Ptr<OpenGymBoxContainer<uint32_t>> box =  CreateObject<OpenGymBoxContainer<uint32_t>> (shape);
  Ptr<UniformRandomVariable> rndVariable = CreateObject<UniformRandomVariable>();
  int rnd;
  Vector position;
  uint32_t min = 0;
  uint32_t max = distance;

  //Cambia posición del nodo de forma aleatoria
  for (uint32_t i = 0; i < nodesNum; i++){
    rnd = rndVariable->GetInteger(min,max);
    box->AddValue(rnd);
    position =  Vector(rnd,0,0);//Nueva posición aleatoria del nodo en el eje x
    Ptr<MobilityModel> mobility = nodes.Get(i)->GetObject<MobilityModel>();
    mobility->SetPosition(position);
  }
  NS_LOG_UNCOND ("Observation -> " << box);
  return box;
}

// el framework retorna el estado del ambiente al agente a cambio de que le envia 
// la siguiente accion a realizar. Igual que en GetObservation, las acciones estan
// codificadas como valores numericos en un contenedor. En este caso solo retorna
// verdadero con el fin de que continue con el ciclo.
bool Actions(Ptr<OpenGymDataContainer> action){
  Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
  NS_LOG_UNCOND ("Acción: " << action);
  return true;
}


void ScheduleNextStateRead(double stepTime, Ptr<OpenGymInterface> openGym){
  Simulator::Schedule (Seconds(stepTime), &ScheduleNextStateRead, stepTime, openGym);
  openGym->NotifyCurrentState();
}

//Recompensa -> el numero de paquetes que fueron recibidos
float GetReward(){
  NS_LOG_UNCOND ("Reward -> " << receivedPackets);
  return receivedPackets;
}

bool GetGameOver(){
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
 
  uint32_t sinkNode = 0; //Nodo fuente
  uint32_t sourceNode = 14;//Nodo destino
  bool tracing = true;
  double simulationTime = 20.0; // en segundos
  double startTraffic = 10.0; 
  double step = 1.0;
  uint32_t port = 5555;

  NS_LOG_UNCOND ("==============Simulation ===============================");

  CommandLine cmd;
  cmd.AddValue("distance","Distancia",distance);
  cmd.AddValue("packetDim","Tamaño x paquete",packetDim);
  cmd.AddValue("openGymPort","Puerto",port);
  cmd.AddValue("rNode","Nodo receptor",sinkNode);
  cmd.AddValue("sNode","Nodo emisor",sourceNode);
  cmd.AddValue("simTime","Tiempo de la simulación",simulationTime);
  cmd.AddValue("stTraffic","Inicio envio de paquetes",startTraffic);
  cmd.Parse(argc, argv);
  
  //Interfaz de OpenIa
  Ptr<OpenGymInterface> openGym = CreateObject<OpenGymInterface> (port);
  openGym->SetGetObservationSpaceCb( MakeCallback (&GetObservationSpace) );
  openGym->SetGetObservationCb( MakeCallback (&GetObservation) );
  openGym->SetGetActionSpaceCb( MakeCallback (&GetActionSpace) );
  openGym->SetExecuteActionsCb( MakeCallback (&Actions) );
  openGym->SetGetRewardCb( MakeCallback (&GetReward) );
  openGym->SetGetGameOverCb( MakeCallback (&GetGameOver) );

  //Convert to time object
  Time interPacketInterval = Seconds (step);

  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));

//contenedor de nodos
  nodes.Create (nodesNum);

//Wifi set up(red inalambrica)
  WifiHelper wifi;
  YansWifiPhyHelper wifiPhy; 
  wifiPhy.Set ("RxGain", DoubleValue (-10) );
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  WifiMacHelper wifiMac;
  wifi.SetStandard (WIFI_STANDARD_80211b);//Protocolo de red
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  wifiMac.SetType ("ns3::AdhocWifiMac");//Configuración a modo adhoc
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);


  //Clase auxiliar utilizada para asignar posiciones y modelos de movilidad a los nodos.
  MobilityHelper mobility; 
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (distance),
                                 "DeltaY", DoubleValue (distance),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  //Clase auxiliar que agrega enrutamiento OLSR a los nodos.
  //Esto es muy util ya el protocolo OLSR (Optimized Link State Routing)
  //permite un enrutamiento proactivo para establecer conexiones entre los
  //nodos en un red inalambrica ad hoc.
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
  Ptr<Socket> recvSink = Socket::CreateSocket (nodes.Get (sinkNode), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
  Ptr<Socket> source = Socket::CreateSocket (nodes.Get (sourceNode), tid);
  InetSocketAddress remote = InetSocketAddress (i.GetAddress (sinkNode, 0), 80);
  source->Connect (remote);

//Trazado de rutas
  if (tracing == true)
    {
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("wifi-adhoc.tr"));
      wifiPhy.EnablePcap ("wifi-simple-adhoc-grid", devices);
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("wifi-adhoc.routes", std::ios::out);
      olsr.PrintRoutingTableAllEvery (Seconds (2), routingStream);
      Ptr<OutputStreamWrapper> neighborStream = Create<OutputStreamWrapper> ("wifi-adhoc.neighbors", std::ios::out);
      olsr.PrintNeighborCacheAllEvery (Seconds (2), neighborStream);
    }

//generacion de tráfico
  Simulator::Schedule (Seconds (startTraffic), &GenerateTraffic,
                       source, packetDim, packetNum, interPacketInterval);
  NS_LOG_UNCOND ("Distance -> "<<distance);
  NS_LOG_UNCOND ("Size x packet ->"<<packetDim);
  NS_LOG_UNCOND ("packets number ->"<<packetNum);
  NS_LOG_UNCOND ("Testing from node " << sourceNode << " to " << sinkNode << " with grid distance " << distance);

  Simulator::Schedule (Seconds(1.0), &ScheduleNextStateRead, step, openGym);//First Observation and step
  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run ();
  openGym->NotifySimulationEnd();
  Simulator::Destroy ();
  NS_LOG_UNCOND ("==============End Simulation ===============================");
  return 0;
}
