This is the codebase for the host-side dashboard to control the Railbot. It has three components (which have separate installation requirements. 

1. An MQTT broker with a TCP and a WebSocket interface. We use Aedes (https://github.com/moscajs/aedes), but it should not be too hard to switch to other MQTT brokers if necessary. 
2. An Express-based web-server which serves the dashboard, map tiles, and blockly programming blocks. Currently, the map tiles are hardcoded to be in the Greater Lafayette area. Flexible global mapping is left for future work. Blockly mission programming is currently not functional and is not meant to be operational for Phase 2.
3. A web-front-end that shows the dashboard elements that include live maps, telemetry display blocks, and an interactive command terminal. The dashboard connects to the MQTT broker over web sockets.

# Pre-Requisites
1. Install aedes (https://github.com/moscajs/aedes)
2. Install node.js and npm (https://nodejs.dev/)

# Installation
1. Run `npm install` in the `frontend` directory. This installs the Express webserver framework module. 
2. Run `npm install` in the `frontend/dist` directory. This installs the Blockly, and MQTT framework code.
3. All map tiles and map-serving javascript is currently hardcoded in the `dist/leaflet` and `dist/maps` subdirectories.

# Run the dashboard
Note that for the dashboard to be meaningful, a Railbot must be running with the ROS MQTT Bridge node that is connected to the same MQTT broker. 

1. **Start the MQTT broker.** On windows hosts, run `broker-start.bat`. If using a Mac or Linux host, you may directly use the command `aedes --protos tcp ws --host <hostname or ip address> --verbose`. 
2. **Start the web server.** Run `node dashserve.js`. By default, the server listens on `http:\\localhost:4040`.
3. Launch a browser and navigate to `http:\\localhost:4040`
