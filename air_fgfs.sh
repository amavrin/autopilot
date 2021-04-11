/Applications/FlightGear.app/Contents/MacOS/fgfs \
--timeofday=noon \
--geometry=800x600 \
--prop:/nasal/local_weather/enabled=false \
--metar=XXXX 012345Z 15003KT 19SM FEW072 FEW350 25/07 Q1028 NOSIG \
--prop:/environment/weather-scenario=Core high pressure region \
--disable-rembrandt \
--disable-terrasync \
--aircraft=org.flightgear.fgaddon.stable_2018.g109 \
--httpd=8080 \
--telnet=1024 \
--airport=PHNL \
--runway=22R \
--offset-distance=3 \
--on-ground=false \
--enable-freeze \
--vc=90 \
--altitude=1300 \
--prop:/engines/engine[0]/rpm=1000 \
--generic=socket,out,4,localhost,10000,udp,output \
--generic=socket,in,4,localhost,10001,udp,input
