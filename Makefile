remote:
	idf.py -DDEVICE=remote build flash monitor

v1_motor:
	idf.py -DDEVICE=v1 build flash monitor

v2_motor:
	idf.py -DDEVICE=v2 build flash monitor