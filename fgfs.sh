/Applications/FlightGear.app/Contents/MacOS/fgfs \
--timeofday=noon \
--prop:/nasal/local_weather/enabled=false \
--metar=XXXX 012345Z 15003KT 19SM FEW072 FEW350 25/07 Q1028 NOSIG \
--prop:/environment/weather-scenario=Core high pressure region \
--disable-rembrandt \
--disable-terrasync \
--geometry=800x600 \
--aircraft=org.flightgear.fgaddon.stable_2018.g109 \
--airport=PHNL \
--runway=22L \
--httpd=8080 \
--telnet=1024 \
--generic=socket,out,4,localhost,10000,udp,output \
--generic=socket,in,4,localhost,10001,udp,input

