D:

cd D:\Program Files\FlightGear 2020.3

SET FG_ROOT=D:\Program Files\FlightGear 2020.3\data

.\\bin\fgfs.exe --fdm=null --native-fdm=socket,in,30,localhost,5502,udp --native-ctrls=socket,out,30,localhost,5505,udp --enable-terrasync --prop:/sim/rendering/shaders/quality-level=0 --aircraft=Rascal110-Electric-YASim --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --airport=KSFO --runway=10L --altitude=7224 --heading=113 --offset-distance=4.72 --offset-azimuth=0
