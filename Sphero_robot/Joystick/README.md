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
Na kraju je potrebno pokrenuti skriptu za upravljanje sfero robotom:
```
python joy_sphero_class.py
```

Naredbe na kontroleru su:
- Lijeva gljivica - smjer kretanja
- X - pojačanje plave boje
- A - pojačanje zelene boje
- B - pojačanje crvene boje
- Y - Resetiranje svih boja na nulu
- R1 + (X,A,B) - smanjenje intenziteta odabrane boje
- L1 - zakret za 40° 
- R2 - Povećanje brzine
- L2 - Smanjenje brzine
- R3 - maksimalna brzina


Za više informacija možete posjetiti [ros wiki](http://wiki.ros.org/joy).
