#lsmod | grep -q xr17v35x
#lsmod | grep xeno_16550A

echo -n "Installing RS-485 Driver... "
sudo insmod forceRS485diver/xr17v35x.ko
echo "[OK]"
for i in {1..100}
do
	echo -ne \\r"Waiting for driver installed... $i%"
	sleep 0.2s
done
echo -e \\r"Waiting for driver installed... [OK]"

echo -n "Unloading serial driver... "
sudo setserial /dev/ttyXR0 uart none
sudo setserial /dev/ttyXR1 uart none
sudo setserial /dev/ttyXR2 uart none
sudo setserial /dev/ttyXR3 uart none
echo "[OK]"

echo -n "Loading xenomai serial driver..."
sudo modprobe xeno_16550A mem=0xf0400000,0xf0400400,0xf0400800,0xf0400c00 irq=11,11,11,11 baud_base=7812500,7812500,7812500,7812500 tx_fifo=256,256,256,256
echo "[OK]"
echo "done"



