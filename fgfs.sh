/Applications/FlightGear.app/Contents/MacOS/fgfs \
--timeofday=noon \
--prop:/nasal/local_weather/enabled=false \
--metar=XXXX 012345Z 15003KT 19SM FEW072 FEW350 25/07 Q1028 NOSIG \
--prop:/environment/weather-scenario=Core high pressure region \
--disable-rembrandt \
--disable-terrasync \
--aircraft=org.flightgear.fgaddon.stable_2018.g109 \
--airport=PHNL \
--httpd=8080 \
--telnet=1024 \
--generic=socket,out,2,localhost,10000,udp,output \
--generic=socket,in,2,localhost,10001,udp,input

