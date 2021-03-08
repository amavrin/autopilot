DATA_DIR=/Applications/FlightGear.app/Contents/Resources/data
PROTO_DIR=${DATA_DIR}/Protocol

install_proto:
	install input.xml ${PROTO_DIR}
	install output.xml ${PROTO_DIR}
