gnome-terminal -t "m600_ground" -x bash -c "source ~/M600_serial_test/devel/setup.bash;rosrun serial_test m600_ground;exec bash";
gnome-terminal -t "m600_ui" -x bash -c "source ~/M600_serial_test/devel/setup.bash;python ~/M600_serial_test/src/serial_test/script/m600_ob.py;exec bash";

