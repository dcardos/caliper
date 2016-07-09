modprobe -r nf2
modprobe nf2
cpci_reprogram.pl
nf2_download ../../bitfiles/event_sizing2.bit
ifconfig nf2c0 up
ifconfig nf2c1 up
sysctl -w net.ipv4.ip_forward=1
route add -net 10.0.0.3 netmask 255.255.255.255 dev nf2c1
ip neigh add 10.0.0.3 lladdr 00:15:17:63:9f:74 dev nf2c1
ifconfig nf2c0 promisc
ifconfig nf2c1 promisc
route add -net 10.0.0.1 netmask 255.255.255.255 dev nf2c0
ip neigh add 10.0.0.1 lladdr 00:4e:46:32:43:00  dev nf2c0
ifconfig nf2c0 10.0.0.5 netmask 255.255.255.0
../../lib/init_qmonitor 0x4
