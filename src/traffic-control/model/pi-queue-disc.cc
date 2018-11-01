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
    .AddAttribute ("MaxSize",
                   "The maximum number of packets accepted by this queue disc",
                   QueueSizeValue (QueueSize ("25p")),
                   MakeQueueSizeAccessor (&QueueDisc::SetMaxSize,
                                          &QueueDisc::GetMaxSize),
                   MakeQueueSizeChecker ())
    //Self Tuning PI
    .AddAttribute ("STPI",
                   "True to enable Self Tuning PI",
                   BooleanValue (false),
                   MakeBooleanAccessor (&PiQueueDisc::m_isSTPI),
                   MakeBooleanChecker ())
    .AddAttribute ("LinkCapacity",
                   "The STPI Link Capacity",
                   DoubleValue (622),
                   MakeDoubleAccessor (&PiQueueDisc::m_capacity),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Kc",
                   "Filter time constant to smoothen capacity",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&PiQueueDisc::m_Kc),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Knrc",
                   "Filter time constant to smoothen N/R*C",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&PiQueueDisc::m_Knrc),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BPI",
                   "Controls AQM responsiveness",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&PiQueueDisc::m_BPI),
                   MakeDoubleChecker<double> (0,0.85))
    .AddAttribute ("Thc",
                   "Smoothened estimate of C",
                   DoubleValue (0),
                   MakeDoubleAccessor (&PiQueueDisc::m_Thc),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Thnrc",
                   "Smoothened estimate of N/R*C",
                   DoubleValue (0),
                   MakeDoubleAccessor (&PiQueueDisc::m_Thnrc),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("rtt",
                   "estimated round trip time",
                   DoubleValue (0),
                   MakeDoubleAccessor (&PiQueueDisc::m_rtt),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Kp",
                   "PI parameter",
                   DoubleValue (0),
                   MakeDoubleAccessor (&PiQueueDisc::m_Kp),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Ki",
                   "PI parameter",
                   DoubleValue (0),
                   MakeDoubleAccessor (&PiQueueDisc::m_Ki),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("IdleTime",
                   "Router's idle time",
                   TimeValue (Seconds (0.0)),
                   MakeTimeAccessor (&PiQueueDisc::m_idleTime),
                   MakeTimeChecker ())
    .AddAttribute ("IdleStartTime",
                   "Router's idle start time",
                   TimeValue (Seconds (0.0)),
                   MakeTimeAccessor (&PiQueueDisc::m_idleStartTime),
                   MakeTimeChecker ())
    .AddAttribute ("UseEcn",
                   "True to use ECN (packets are marked instead of being dropped)",
                   BooleanValue (false),
                   MakeBooleanAccessor (&PiQueueDisc::m_useEcn),
                   MakeBooleanChecker ())

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

void
PiQueueDisc::SetQueueLimit (uint32_t lim)
{
  NS_LOG_FUNCTION (this << lim);
  SetMaxSize (QueueSize (GetMaxSize ().GetUnit (), lim));
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

  uint32_t dropType = DTYPE_NONE;
  if ((nQueued >= m_queueLimit)
      || (nQueued + item->GetSize () > m_queueLimit))
    {
      // Drops due to queue limit: reactive
      DropBeforeEnqueue (item, FORCED_DROP);
      dropType = DTYPE_FORCED;
      m_stats.forcedDrop++;
      return false;
    }
  else if (DropEarly (item, nQueued))
    {
      // Early probability drop: proactive
      DropBeforeEnqueue (item, UNFORCED_DROP);
      dropType = DTYPE_UNFORCED;
      m_stats.unforcedDrop++;
      return false;
    }
   if (dropType == DTYPE_UNFORCED)
    {
      if (!m_useEcn || !Mark (item, UNFORCED_MARK))
        {
          NS_LOG_DEBUG ("\t Dropping due to Prob Mark ");
          DropBeforeEnqueue (item, UNFORCED_DROP);
          return false;
        }
      NS_LOG_DEBUG ("\t Marking due to Prob Mark ");
    }
  else if (dropType == DTYPE_FORCED)
    {
      if (!m_useEcn || !Mark (item, FORCED_MARK))
        {
          NS_LOG_DEBUG ("\t Dropping due to Forced Mark ");
          DropBeforeEnqueue (item, FORCED_DROP);
          return false;
        }
      NS_LOG_DEBUG ("\t Marking due to Forced Mark ");
    }

  // No drop
  bool retval = GetInternalQueue (0)->Enqueue (item);

  //Self Tuning PI
  if (m_idle)
    {
      Time now = Simulator :: Now ();
      m_idleTime = (now - m_idleStartTime);
    }
  m_idle = false;
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

  //Self-Tuning PI
  if (m_isSTPI)
    {
      m_oldThc = 0;
      m_oldThnrc = 0;
      m_idle = true;
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
  if (m_isSTPI)
    {
      m_routerBusyTime = uint32_t ((((Simulator :: Now ()) - m_oldRoutBusyTime) - m_idleTime).GetSeconds ());
      m_capacity = m_departedPkts / m_routerBusyTime;
      m_Thc = ((m_oldThc * (1 - m_Kc)) + (m_Kc * m_capacity));
      m_Thnrc = (m_oldThnrc * (1 - m_Knrc) + (m_Knrc * (std :: sqrt (p / 2))));
      m_rtt = (((m_Thnrc / m_Thc)) / (std :: sqrt (p / 2)));
      m_Kp = (2 * m_BPI * (std :: sqrt ((m_BPI * m_BPI) + 1)) * m_Thnrc ) / (m_rtt * m_Thc);
      m_Ki = ((2 * m_Thnrc) / m_rtt) * m_Kp;
      m_departedPkts = 0;
      m_idleTime = NanoSeconds (0);
      m_oldRoutBusyTime = Simulator :: Now ();
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
      //Self-Tuning PI
      m_idleStartTime = Simulator::Now ();
      m_idle = true;
      return 0;
    }
  else
    {

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

    }

  if (GetNInternalQueues () != 1)
    {
      NS_LOG_ERROR ("PiQueueDisc needs 1 internal queue");
      return false;
    }

  return true;
}

} //namespace ns3
