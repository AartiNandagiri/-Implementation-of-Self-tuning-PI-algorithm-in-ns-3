/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 NITK Surathkal
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Priya S Tavarmani <priyast663@gmail.com>
 *          Viyom Mittal <viyommittal@gmail.com>
 *          Mohit P. Tahiliani <tahiliani@nitk.edu.in>
 */

/*
 * PORT NOTE: This code was ported from ns-2.36rc1 (queue/pi.cc).
 * Most of the comments are also ported from the same.
 */

#include "ns3/log.h"
#include "ns3/enum.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/simulator.h"
#include "ns3/abort.h"
#include "pi-queue-disc.h"
#include "ns3/drop-tail-queue.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("PiQueueDisc");

NS_OBJECT_ENSURE_REGISTERED (PiQueueDisc);

TypeId PiQueueDisc::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PiQueueDisc")
    .SetParent<QueueDisc> ()
    .SetGroupName ("TrafficControl")
    .AddConstructor<PiQueueDisc> ()
    /*.AddAttribute ("Mode",
                   "Determines unit for QueueLimit",
                   EnumValue (QUEUE_DISC_MODE_PACKETS),
                   MakeEnumAccessor (&PiQueueDisc::SetMode,
                                     &PiQueueDisc::GetMode),
                   MakeEnumChecker (QUEUE_DISC_MODE_BYTES, "QUEUE_DISC_MODE_BYTES",
                                    QUEUE_DISC_MODE_PACKETS, "QUEUE_DISC_MODE_PACKETS"),
                   TypeId::DEPRECATED,
                   "Use the MaxSize attribute instead")*/
    .AddAttribute ("MeanPktSize",
                   "Average of packet size",
                   UintegerValue (500),
                   MakeUintegerAccessor (&PiQueueDisc::m_meanPktSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("QueueRef",
                   "Desired queue size",
                   DoubleValue (50),
                   MakeDoubleAccessor (&PiQueueDisc::m_qRef),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("A",
                   "Value of alpha",
                   DoubleValue (0.00001822),
                   MakeDoubleAccessor (&PiQueueDisc::m_a),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("B",
                   "Value of beta",
                   DoubleValue (0.00001816),
                   MakeDoubleAccessor (&PiQueueDisc::m_b),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("W",
                   "Sampling frequency",
                   DoubleValue (170),
                   MakeDoubleAccessor (&PiQueueDisc::m_w),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("QueueLimit",
                   "Queue limit in bytes/packets",
                   DoubleValue (50),
                   MakeDoubleAccessor (&PiQueueDisc::SetQueueLimit),
                   MakeDoubleChecker<double> ())
    //Self Tuning PI
    .AddAttribute("STPI",
                  "True to enable Self Tuning PI",
                  BooleanValue (false),
                  MakeBooleanAccessor (&PiQueueDisc::m_isSTPI),
                  MakeBooleanChecker ())
    .AddAttribute("LinkCapacity",
                  "The STPI Link Capacity",
                  DoubleValue (622),
                  MakeDoubleAccessor (&PiQueueDisc::m_capacity),
                  MakeDoubleChecker<double> ())
    .AddAttribute("Kc",
                  "Filter time constant to smoothen capacity",
                  DoubleValue (0.5),
                  MakeDoubleAccessor (&PiQueueDisc::m_Kc),
                  MakeDoubleChecker<double> ())
    .AddAttribute("Knrc",
                  "Filter time constant to smoothen N/R*C",
                  DoubleValue (0.5),
                  MakeDoubleAccessor (&PiQueueDisc::m_Knrc),
                  MakeDoubleChecker<double> ())
    .AddAttribute("BPI",
                  "Controls AQM responsiveness",
                  DoubleValue(0.5),
                  MakeDoubleAccessor(&PiQueueDisc::m_BPI),
                  MakeDoubleChecker<double>(0,0.85))
    .AddAttribute("Thc",
                  "Smoothened estimate of C",
                  DoubleValue (0),
                  MakeDoubleAccessor (&PiQueueDisc::m_Thc),
                  MakeDoubleChecker<double> ())
    .AddAttribute("Thnrc",
                  "Smoothened estimate of N/R*C",
                  DoubleValue (0),
                  MakeDoubleAccessor (&PiQueueDisc::m_Thnrc),
                  MakeDoubleChecker<double> ())
    .AddAttribute("rtt",
                  "estimated round trip time",
                  DoubleValue (0),
                  MakeDoubleAccessor (&PiQueueDisc::m_rtt),
                  MakeDoubleChecker<double> ())
    .AddAttribute("Kp",
                  "PI parameter",
                  DoubleValue (0),
                  MakeDoubleAccessor (&PiQueueDisc::m_Kp),
                  MakeDoubleChecker<double> ())
    .AddAttribute("Ki",
                  "PI parameter",
                  DoubleValue (0),
                  MakeDoubleAccessor (&PiQueueDisc::m_Ki),
                  MakeDoubleChecker<double> ())
    .AddAttribute("IdleTime",
                  "Router's idle time",
                  TimeValue (Seconds (0.0)),
                  MakeTimeAccessor (&PiQueueDisc::m_idleTime),
                  MakeTimeChecker ())
    .AddAttribute("IdleStartTime",
                  "Router's idle start time",
                  TimeValue (Seconds (0.0)),
                  MakeTimeAccessor (&PiQueueDisc::m_idleStartTime),
                  MakeTimeChecker ())
    
  ;

  return tid;
}

PiQueueDisc::PiQueueDisc ()
  : QueueDisc ()
{
  NS_LOG_FUNCTION (this);
  m_uv = CreateObject<UniformRandomVariable> ();
  m_rtrsEvent = Simulator::Schedule (Time (Seconds (1.0 / m_w)), &PiQueueDisc::CalculateP, this);
 
}

PiQueueDisc::~PiQueueDisc ()
{
  NS_LOG_FUNCTION (this);
}

void
PiQueueDisc::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_uv = 0;
  Simulator::Remove (m_rtrsEvent);
  QueueDisc::DoDispose ();
}

/*void
PiQueueDisc::SetMode (QueueDiscMode mode)
{
  NS_LOG_FUNCTION (this << mode);
  if (mode == QUEUE_DISC_MODE_BYTES)
    {
      SetMaxSize (QueueSize (QueueSizeUnit::BYTES, GetMaxSize ().GetValue ()));
    }
  else if (mode == QUEUE_DISC_MODE_PACKETS)
    {
      SetMaxSize (QueueSize (QueueSizeUnit::PACKETS, GetMaxSize ().GetValue ()));
    }
  else
    {
      NS_ABORT_MSG ("Unknown queue size unit");
    }
 // m_mode = mode;
}

PiQueueDisc::QueueDiscMode
PiQueueDisc::GetMode (void) const
{
  NS_LOG_FUNCTION (this);
  return (GetMaxSize ().GetUnit () == QueueSizeUnit::PACKETS ? QUEUE_DISC_MODE_PACKETS : QUEUE_DISC_MODE_BYTES);
 // return m_mode;
}
*/
void
PiQueueDisc::SetQueueLimit (uint32_t lim)
{
  NS_LOG_FUNCTION (this << lim);
  SetMaxSize (QueueSize (GetMaxSize ().GetUnit (), lim));
  //m_queueLimit = lim;
}

uint32_t
PiQueueDisc::GetQueueSize (void)
{
  NS_LOG_FUNCTION (this);
  return GetInternalQueue (0)->GetCurrentSize ().GetValue ();
}


PiQueueDisc::Stats
PiQueueDisc::GetStats ()
{
  NS_LOG_FUNCTION (this);
  return m_stats;
}

int64_t
PiQueueDisc::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uv->SetStream (stream);
  return 1;
}

bool
PiQueueDisc::DoEnqueue (Ptr<QueueDiscItem> item)
{
  NS_LOG_FUNCTION (this << item);

  uint32_t nQueued = GetQueueSize ();
  if ((nQueued >= m_queueLimit)
      || (nQueued + item->GetSize () > m_queueLimit))
    {
      // Drops due to queue limit: reactive
      DropBeforeEnqueue (item, "FORCED_DROP");
      m_stats.forcedDrop++;
      return false;
    }
  else if (DropEarly (item, nQueued))
    {
      // Early probability drop: proactive
      DropBeforeEnqueue (item, "UNFORCED_DROP");
      m_stats.unforcedDrop++;
      return false;
    }

  // No drop
  bool retval = GetInternalQueue (0)->Enqueue (item);
  //Self Tuning PI
  Time now = Simulator :: Now();
  m_idleTime = (now - m_idleStartTime);
  // If Queue::Enqueue fails, QueueDisc::Drop is called by the internal queue
  // because QueueDisc::AddInternalQueue sets the drop callback

  NS_LOG_LOGIC ("\t bytesInQueue  " << GetInternalQueue (0)->GetNBytes ());
  NS_LOG_LOGIC ("\t packetsInQueue  " << GetInternalQueue (0)->GetNPackets ());

  return retval;
}

void
PiQueueDisc::InitializeParams (void)
{
  m_dropProb = 0;
  m_stats.forcedDrop = 0;
  m_stats.unforcedDrop = 0;
  m_qOld = 0;
  //STPI
 
  if(m_isSTPI)
   {
     m_oldThc=0;
     m_oldThnrc=0;
   }
}

bool PiQueueDisc::DropEarly (Ptr<QueueDiscItem> item, uint32_t qSize)
{
  NS_LOG_FUNCTION (this << item << qSize);

  double p = m_dropProb;
  bool earlyDrop = true;
  
  if (GetMaxSize ().GetUnit () == QueueSizeUnit::BYTES)
    {
       p = p * item->GetSize () / m_meanPktSize;
    }
  p = p > 1 ? 1 : p;

  double u =  m_uv->GetValue ();

  if (u > p)
    {
      earlyDrop = false;
    }
  if (!earlyDrop)
    {
      return false;
    }

  return true;
}

void PiQueueDisc::CalculateP ()
{
  NS_LOG_FUNCTION (this);
  double p = 0.0;
  uint32_t qlen = GetQueueSize ();
  //Self Tuning PI (STPI)
  if(m_isSTPI)
    {
      m_routerBusyTime = uint32_t ((((Simulator :: Now()) - m_oldRoutBusyTime) - m_idleTime).GetSeconds ());
      m_capacity = m_departedPkts/m_routerBusyTime;
      m_Thc = ((m_oldThc * (1 - m_Kc)) + (m_Kc * m_capacity));
      m_Thnrc = (m_oldThnrc *(1- m_Knrc) + (m_Knrc * (std :: sqrt (p/2)))) ;
      m_rtt = (((m_Thnrc / m_Thc)) / (std :: sqrt (p/2)));
      m_Kp = (2 * m_BPI * (std :: sqrt ((m_BPI * m_BPI) + 1)) * m_Thnrc )/(m_rtt * m_Thc); 
      m_Ki = ((2 * m_Thnrc)/m_rtt) * m_Kp;
      m_departedPkts = 0;
      m_idleTime = NanoSeconds (0);
      m_oldRoutBusyTime = Simulator :: Now(); 
      if (GetMaxSize ().GetUnit () == QueueSizeUnit::BYTES)
        {
          p = m_Ki * ((qlen * 1.0 / m_meanPktSize) - m_qRef) + m_Kp * (qlen * 1.0 / m_meanPktSize);
          
        }
      else
        {
          p = m_Ki * (qlen - m_qRef) + m_Kp * qlen;
        }

    }
 //PI
 else
 {
   if (GetMaxSize ().GetUnit () == QueueSizeUnit::BYTES)
    {
      p = m_a * ((qlen * 1.0 / m_meanPktSize) - m_qRef) - m_b * ((m_qOld * 1.0 / m_meanPktSize) - m_qRef) + m_dropProb;
    }
   else
    {
      p = m_a * (qlen - m_qRef) - m_b * (m_qOld - m_qRef) + m_dropProb;
    }
}
  p = (p < 0) ? 0 : p;
  p = (p > 1) ? 1 : p;

  m_dropProb = p;
  m_qOld = qlen;
  m_rtrsEvent = Simulator::Schedule (Time (Seconds (1.0 / m_w)), &PiQueueDisc::CalculateP, this);

 
}

Ptr<QueueDiscItem>
PiQueueDisc::DoDequeue ()
{
  NS_LOG_FUNCTION (this);

  if (GetInternalQueue (0)->IsEmpty ())
    {
      NS_LOG_LOGIC ("Queue empty");
        //Self Tuning PI
        //m_idle=1;
        m_idleStartTime = Simulator::Now();
      return 0;
    }
  else
  {
       // m_idle=0;
        Ptr<QueueDiscItem> item = StaticCast<QueueDiscItem> (GetInternalQueue (0)->Dequeue ());
        m_departedPkts++;
        return item;
   }
}

Ptr<const QueueDiscItem>
PiQueueDisc::DoPeek () const
{
  NS_LOG_FUNCTION (this);
  if (GetInternalQueue (0)->IsEmpty ())
    {
      NS_LOG_LOGIC ("Queue empty");
      return 0;
    }

  Ptr<const QueueDiscItem> item = StaticCast<const QueueDiscItem> (GetInternalQueue (0)->Peek ());

  NS_LOG_LOGIC ("Number packets " << GetInternalQueue (0)->GetNPackets ());
  NS_LOG_LOGIC ("Number bytes " << GetInternalQueue (0)->GetNBytes ());

  return item;
}

bool
PiQueueDisc::CheckConfig (void)
{
  NS_LOG_FUNCTION (this);
  if (GetNQueueDiscClasses () > 0)
    {
      NS_LOG_ERROR ("PiQueueDisc cannot have classes");
      return false;
    }

  if (GetNPacketFilters () > 0)
    {
      NS_LOG_ERROR ("PiQueueDisc cannot have packet filters");
      return false;
    }

  if (GetNInternalQueues () == 0)
    {
       // create a DropTail queue
      AddInternalQueue (CreateObjectWithAttributes<DropTailQueue<QueueDiscItem> >
                          ("MaxSize", QueueSizeValue (GetMaxSize ())));
      /*Ptr<Queue> queue = CreateObjectWithAttributes<DropTailQueue> ("Mode", EnumValue (m_mode));
      if (m_mode == QUEUE_DISC_MODE_PACKETS)
        {
          queue->SetMaxPackets (m_queueLimit);
        }
      else
        {
          queue->SetMaxBytes (m_queueLimit);
        }
      AddInternalQueue (queue);*/
    }

  if (GetNInternalQueues () != 1)
    {
      NS_LOG_ERROR ("PiQueueDisc needs 1 internal queue");
      return false;
    }

  /*if (GetInternalQueue (0)->GetMode () != m_mode)
    {
      NS_LOG_ERROR ("The mode of the provided queue does not match the mode set on the PiQueueDisc");
      return false;
    }

  if ((m_mode == QUEUE_DISC_MODE_PACKETS && GetInternalQueue (0)->GetMaxPackets () < m_queueLimit)
      || (m_mode == QUEUE_DISC_MODE_BYTES && GetInternalQueue (0)->GetMaxBytes () < m_queueLimit))
    {
      NS_LOG_ERROR ("The size of the internal queue is less than the queue disc limit");
      return false;
    }*/

  return true;
}

} //namespace ns3
