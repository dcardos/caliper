ifconfig eth1 down
ifconfig eth2 down
ifconfig eth1 up
ifconfig eth1 10.0.0.2 netmask 255.255.255.0
ifconfig eth2 up
ifconfig eth2 10.0.0.3 netmask 255.255.255.0
sysctl -w net.ipv4.ip_forward=1
route add -net 10.0.0.4 netmask 255.255.255.255 dev eth1
route add -net 10.0.0.1 netmask 255.255.255.255 dev eth1
ip neigh add 10.0.0.4 lladdr 00:4e:46:32:43:00 dev eth1
ip neigh add 10.0.0.1 lladdr 00:1e:4f:19:9a:b1 dev eth1
ifconfig eth1 promisc
ifconfig eth2 promisc
