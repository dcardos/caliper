modprobe -r nf2
modprobe nf2
cpci_reprogram.pl
nf2_download ../../bitfiles/event_sizing2.bit
ifconfig nf2c0 up
ifconfig nf2c1 up
sysctl -w net.ipv4.ip_forward=1
../../lib/init_qmonitor 0x4
ifconfig nf2c0 10.0.0.5 netmask 255.255.255.0
