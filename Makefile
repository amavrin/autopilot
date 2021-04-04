DATA_DIR=/Applications/FlightGear.app/Contents/Resources/data
AIRCRAFT_DATA_DIR="/Users/alexey.mavrin/Library/Application Support/FlightGear/aircraft-data"
G109_DATA_FILE="org.flightgear.fgaddon.stable_2018.g109.xml"
PROTO_DIR=${DATA_DIR}/Protocol

install_proto:
	install input.xml ${PROTO_DIR}
	install output.xml ${PROTO_DIR}

install_data:
	install g109-data.xml ${AIRCRAFT_DATA_DIR}/${G109_DATA_FILE}
