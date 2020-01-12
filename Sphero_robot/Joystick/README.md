Za ručno upravljanje sfernim robotom Sphero SPRK+ koristi se kontroler Logitech F710.

Prije svega potrebno je pravilno konfigurirati kontroler na Vaše računalo.

Naredbe za konfiguraciju kontrolera na su:
```
ls /dev/input/                   # kontroler je jedan od "jsX" pri čemu je X neki broj
sudo jstest /dev/input/jsX       # testiranje je li to Vaš kontroler
sudo chmod a+rw /dev/input/jsX   # konfiguracija kontrolera
```

U nastavku slijede naredbe za pokretanje "Joy" čvora(engl. *Node*) u ROS-u:
```
roscore
rosparam set joy_node/dev "/dev/input/jsX"
rosrun joy joy_node
rostopic echo joy
```
Na koncu je za upravljanje potrebno pokrenuti skriptu za upravljanje sfero robotom:
```
python joy_sphero_class.py
```

Za više informacija možete posjetiti [ros wiki](http://wiki.ros.org/joy).
