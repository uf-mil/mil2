#sleep 10s
echo "discoverable on" | bluetoothctl
rfcomm watch hci0 1 python3 main.py
