ifconfig nf2c0 down
ifconfig eth1 down
cd ../../driver/kernel && make clean && cd -
../../lib/cpci_reprogram.pl
sleep 3
../../lib/nf2_download ../../bitfiles/netthreads_nooutqueues.bit
sleep 3
make -C ../../pktgen
make -C ../../driver/kernel
make -C ../../netthreads/src/bench/precisegen
make -C ../../driver/kernel modprobe
../../netthreads/loader/loader -i ../../netthreads/src/bench/precisegen/precisegen.instr.mif
../../netthreads/loader/loader -d ../../netthreads/src/bench/precisegen/precisegen.data.mif -nodebug
ifconfig nf2c0 up
ifconfig eth1 up
ifconfig nf2c0 10.0.0.4 netmask 255.255.255.0
ifconfig eth1 10.0.0.1 netmask 255.255.255.0
sysctl -w net.ipv4.ip_forward=1
route add -net 10.0.0.3 netmask 255.255.255.255 dev nf2c0
route add -net 10.0.0.2 netmask 255.255.255.255 dev nf2c0
ip neigh add 10.0.0.3 lladdr 00:15:17:63:a0:c6   dev nf2c0
ip neigh add 10.0.0.2 lladdr 00:1e:4f:18:5f:bb  dev nf2c0
ifconfig nf2c0 promisc
ifconfig eth1 promisc
