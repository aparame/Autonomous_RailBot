//import { connect } from 'mqtt';

//model state
var speed = 0;
var odom = 0;
var range = 3;
var temp = 38;
var loraRSSI = 80;
var mymap;
var mymarker;
var botPosition;
var mqttOptions = {

}
var client  = mqtt.connect('ws://192.168.1.125:3000')
//var client = mqtt.connect('ws://192.168.1.125:3000');

function onLoad() {

    mymap = L.map('mapid').setView([40.433358,-86.897049], 14);

    L.tileLayer('maps/{z}/{x}/{y}.png', {maxZoom: 16}).addTo(mymap);
    mymarker = L.marker([40.422031, -86.894977]).addTo(mymap);

   
}

client.on('connect', function () {
  client.subscribe('ros2host_speed', function (err) {
    if (!err) {
        console.log("Subscribe success: speed.")
    }
  })
  client.subscribe('ros2host_info', function (err) {
    if (!err) {
        console.log("Subscribe success: info.")
    }
  })
  client.subscribe('ros2host_temp', function (err) {
    if (!err) {
        console.log("Subscribe success: temp.")
    }
  })
  client.subscribe('ros2host_range', function (err) {
    if (!err) {
        console.log("Subscribe success: range.")
    }
  })
  client.subscribe('ros2host_odom', function (err) {
    if (!err) {
        console.log("Subscribe success: odom.")
    }
  })
  client.subscribe('ros2host_rssi', function (err) {
    if (!err) {
        console.log("Subscribe success: RSSI.")
    }
  })
  client.subscribe('ros2host_latlong', function (err) {
    if (!err) {
        console.log("Subscribe success: Lat/Long.")
    }
  })
})

client.on('message', function (topic, message) {
  // message is Buffer
  console.log("on topic " + topic + ": received message : " + message.toString())
  //client.end()
  commonMessageProcessing(topic,message);
  
})

function commonMessageProcessing(topic,message) {
  let messageVal = JSON.parse(message);
  if(topic =="ros2host_latlong") {
        let latLngArray = eval(message.toString());
        mymarker.setLatLng(latLngArray);

    }
    else if(topic=="ros2host_info") {
      // Special case: Do nothing; no dedicated display
    }
    else {
        var elem = document.getElementById(topic);
        if (topic =="ros2host_temp") messageVal.data = messageVal.data + "&deg;C";
        elem.innerHTML = messageVal.data;
    }
    addToLog(topic,message, "left");
}

function addToLog(topic,message, leftRight) {
    let colors = [];
    colors["ros2host_temp"] = "bg-blue-200";
    colors["ros2host_speed"] = "bg-green-200";
    colors["ros2host_odom"] = "bg-red-200";
    colors["host2ros"] = "bg-gray-200";
    colors["ros2host_latlong"] = "bg-yellow-200";
    colors["ros2host_info"] = "bg-gray-200";

    let leftRightClass;
    if (leftRight =="left") leftRightClass = "self-start";
    else leftRightClass = "self-end"
    var elem = document.getElementById("commandLog");
    let messageVal = JSON.parse(message);
    //console.log("extracted value " + messageVal.data);
    elem.innerHTML += `<div class="m-2 ${leftRightClass} w-max  p-2 ${colors[topic]} rounded">${topic}: ${messageVal.data} </div>`;
}

function sendUserCommand() {
    var entry = document.getElementById("userCommand");
    if( entry.value != "") publishCommand(entry.value);
    entry.value = '';
}

function publishCommand(command) {
    let jsonObject = `{"data" : "${command}"}`;
    client.publish("host2ros", jsonObject);
    //console.log(jsonObject);
    addToLog("host2ros",jsonObject,"right")
}