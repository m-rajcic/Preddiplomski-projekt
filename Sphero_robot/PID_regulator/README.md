Regulator je dobiven Ziegler-Nicholsonovom metodom. Prije korištenja u pravom sustavu je simuliran u Simunlinku. Nakon toga je diskretiziran Tustinovim postupkom te takav koristi u skripti.
Parametri PID regulatora su:
- proporcionalno pojačanje K_p = 0.6
- integralno pojačanje K_i = 0.00005
- derivativno pojačanje K_d = 0
- vrijeme diskretizacije T = 150 ms

Skripta koja pokreće Sphero robote prema cilju je: sphero_final.py 
