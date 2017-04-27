#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

#define DELAY_TRIGGER_THRESH 100

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), cwnd(10), successful_acks_received(0),
  last_delay_triggered(0)
{
  debug_ = true;
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << cwnd << endl;
  }

  return cwnd;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  /* Default: take no action */
  if (timestamp_ack_received - send_timestamp_acked > DELAY_TRIGGER_THRESH
    && timestamp_ack_received - last_delay_triggered > 2 * DELAY_TRIGGER_THRESH) {
    if (cwnd > 1) cwnd /= 2;
    last_delay_triggered = timestamp_ack_received;
    successful_acks_received = 0;
  } else {
    successful_acks_received++;
    if (successful_acks_received == cwnd) {
      cwnd++; 
      successful_acks_received = 0;
    }
  }

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 50; /* timeout of one second */
}
