ifconfig nf2c0 down
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
ifconfig nf2c0 10.0.0.4 netmask 255.255.255.0
