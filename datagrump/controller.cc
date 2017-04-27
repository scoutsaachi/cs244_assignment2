#include <iostream>
#include <math.h>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

#define DELAY_TRIGGER_THRESH 100

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), last_drop_time(0), wmax(20.0),
  beta(0.65), C(4), last_window(0), alpha(1)
{
  last_window = (1 - beta) * wmax;
  debug_ = true;
}

double Controller::window_size_ntrunc( void )
{
  double K = beta * wmax / C;
  K = cbrt(K);
  uint64_t t = timestamp_ms() - last_drop_time;
  double t_d = (t / 1000.0);
  double wcubic = (C * pow(t_d - K, 3.0)) + wmax;
  if (debug_) {
	cerr << "K = " << K << ", t = " << t << ", t_d = " << t_d << ", last_window = " << last_window << ", wcubic = " << wcubic << endl;
  }
  if (wcubic < 1.0) {
    wcubic = 1.0;
  }

  double smin = 1.0 / (alpha * last_window);
  double window_diff = wcubic - last_window;
  if (window_diff < 0) {
    cerr << "resetting" << endl;
    //last_window = wcubic;
    window_diff = 0.0;
  }

  if (window_diff < smin) {
    if (debug_) {
       cerr << "smaller than smin" << endl;
    }
    wcubic = last_window + smin;
  }else {
    cerr << "driven cubic" << endl;
  }

  return wcubic;
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  unsigned int cwnd = (unsigned int) window_size_ntrunc();

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
  uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
  uint64_t t = timestamp_ack_received - last_drop_time;
  if (t > 2 * DELAY_TRIGGER_THRESH
    && rtt > DELAY_TRIGGER_THRESH) {
    if (debug_) {
    	cerr << "cut" << endl;
    }
    wmax = window_size_ntrunc();
    last_drop_time = timestamp_ack_received;
    last_window = (1 - beta) * wmax;
  } else {
    last_window = window_size_ntrunc();
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
