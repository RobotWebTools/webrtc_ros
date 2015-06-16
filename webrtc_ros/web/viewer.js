var params = {};
if(window.location.search.length > 1) {
  var query_string = decodeURIComponent(window.location.search.substring(1));
  var parameter_key_values = query_string.split("&");
  for(i in parameter_key_values) {
    var key_value = parameter_key_values[i];
    if(key_value.length > 0) {
      var key_value_arr = key_value.split("=", 2);
      if(key_value_arr.length == 2) {
        params[key_value_arr[0]] = key_value_arr[1];
      }
      else {
        params[key_value_arr[0]] = "";
      }
    }
  }
}

var connection = WebrtcRos.createConnection();
connection.onConfigurationNeeded = function() {
  var configBuilder = connection.configureBuilder();
  var config_box = document.getElementById("config-box");
  config_box.innerHTML = "<h1>Configuration</h1>";
  if(params["subscribed_video_topic"] && params["subscribed_video_topic"].length > 0) {
    config_box.innerHTML += "Subscribed Video Topic: <b>" + params["subscribed_video_topic"] + "</b>";
    configBuilder.addRemoteStream("subscribed", "subscribed_video", "ros_image:"+params["subscribed_video_topic"]);
  }
  else
    config_box.innerHTML += "Not Subscribed to Video"
  config_box.innerHTML += "<br>"
  if(params["published_video_topic"] && params["published_video_topic"].length > 0) {
    config_box.innerHTML += "Published Video Topic: <b>" + params["published_video_topic"] + "</b>";
  }
  else
    config_box.innerHTML += "Not Publishing Video"

  var sendVideo = params["published_video_topic"] && params["published_video_topic"].length > 0;
  if(sendVideo) {
    configBuilder.addLocalStream({"video": sendVideo}, "ros_image:"+params["published_video_topic"]);
  }
  configBuilder.send();
};
connection.connect();

connection.onRemoteStreamAdded = function(event) {
  var remoteVideoElement = document.getElementById("remote-video");
  remoteVideoElement.src = URL.createObjectURL(event.stream);
  remoteVideoElement.play();
}
