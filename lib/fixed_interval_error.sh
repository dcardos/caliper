#!/bin/bash

set -e
set -u

if [[ $# -ne 2 ]]; then
    echo <<EOF
Usage: $0 PCAP_FILE INTERVAL

PCAP_FILE - A pcap file of events from the NetFPGA buffer monitoring
            router.
INTERVAL - The correct fixed interval between packet
            transmissions. Units is nanoseconds
EOF
    exit 1
fi

PCAP_FILE="$1"
INTERVAL="$2"

/srv/nfnet/tools/evtcap/rcv_evts2 -o $PCAP_FILE \
    | egrep '^S ' \
    | /srv/nfnet/tools/analyze/delta.py -f 1 -q \
    | /srv/nfnet/pktgen/scripts/avg_error.py $INTERVAL
