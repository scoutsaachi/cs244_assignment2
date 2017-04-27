#include <iostream>
#include <math.h>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

#define DELAY_TRIGGER_THRESH 100

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), last_drop_time(0), wmax(20),
  beta(0.5), C(3.5), last_window(0)

{
  last_window = (1 - beta) * wmax;
  debug_ = true;
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  double K = beta * wmax / C;
  K = cbrt(K);
  uint64_t t = timestamp_ms() - last_drop_time;
  double t_d = (t / 1000.0);
  double wcubic = (C * pow(t_d - K, 3.0)) + (double)wmax;
  cerr << "K = " << K << ", t = " << t << ", t_d = " << t_d << ", last_window = " << last_window << ", wcubic = " << wcubic << endl;
  if (wcubic < 1.0) {
    wcubic = 1.0;
  }

  double smin = 1 / last_window;
  double window_diff = wcubic - last_window;
  if (window_diff < 0) {
    last_window = wcubic;
    window_diff = 0.0;
  }

  if (window_diff < smin) {
    cerr << "smaller than smin" << endl;
    wcubic = last_window + smin;
  }

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
   << " window size is " << (unsigned int) wcubic << endl;
  }

  return (unsigned int)wcubic;
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
    cerr << "cut" << endl;
    wmax = window_size();
    last_drop_time = timestamp_ack_received;
    last_window = (1 - beta) * wmax;
  } else {
    last_window = window_size();
  }

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
   << " received ack for datagram " << sequence_number_acked
   << " (send @ time " << send_timestamp_acked
   << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
   << endl;
  }
}

void Controller::timeout_occured( void ) {
  // cwnd = 1;
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 50; /* timeout of one second */
}
