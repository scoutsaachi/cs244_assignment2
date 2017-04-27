#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), cwnd(10), counter(0),
  prev_rtt(0), min_rtt(0), rtt_diff(0),
  updateParameter(1)
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

void Controller::updateAcks(int n) {
  counter+=n;
  while (counter >= ((int)cwnd)) {
    cwnd++;
    counter-=cwnd;
  }

  while (cwnd > 0 && counter <= (-1 * ((int)cwnd))) {
    cwnd--;
    counter+= cwnd;
  }

  if (cwnd <= 0) {
    cwnd = 1;
    counter = 0;
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
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
   << " received ack for datagram " << sequence_number_acked
   << " (send @ time " << send_timestamp_acked
   << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
   << endl;
  }
  long long new_rtt = timestamp_ack_received - send_timestamp_acked;
  if (min_rtt == 0) {
    min_rtt = new_rtt;
    prev_rtt = new_rtt;
    return;
  }

  if (new_rtt < min_rtt) {
    min_rtt = new_rtt;
  }

  double new_rtt_diff = new_rtt - prev_rtt;
  prev_rtt = new_rtt;

  if (new_rtt > rtt_high) {
    updateParameter = 1;
    cerr << "outlier occurs" << endl;
    // int counterDecrease = cwnd * (1 - (beta * (1 - (rtt_high / ((double)(new_rtt))))));
    // updateAcks(-4 * counterDecrease);
    cwnd /= 2;
    if (cwnd == 0) cwnd = 1;
    counter = 0;
    return;
  }

  rtt_diff = ((1.0 - alpha) * rtt_diff) + (alpha * new_rtt_diff);
  double normalized_gradient = rtt_diff / min_rtt;
  cerr << "new rtt: " << new_rtt << "new_rtt_diff = " << new_rtt_diff << ", min_rtt = " << min_rtt << ", rtt_diff = " << rtt_diff << ", normalized_gradient = " << normalized_gradient << endl;
  if (normalized_gradient <= 0) {
    cerr << "Adding one to window" << endl;
    updateAcks(4 * updateParameter);
    updateParameter++;
  } else {
    updateParameter = 1;
    cerr << "Decreasing from gradient by factor of " << (1 - (beta * normalized_gradient)) <<  endl;
    int counterDecrease = cwnd * (1 - (beta * normalized_gradient));
    updateAcks(-1 * counterDecrease);
  }

  
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 1000; /* timeout of one second */
}
