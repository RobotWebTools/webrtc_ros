var params = {};
if(window.location.search.length > 1) {
    var query_string = decodeURIComponent(window.location.search.substring(1));
    var parameter_key_values = query_string.split("&");
    for(var i in parameter_key_values) {
	if(parameter_key_values.hasOwnProperty(i)) {
	    var key_value = parameter_key_values[i];
	    if(key_value.length > 0) {
		var key_value_arr = key_value.split("=", 2);
		if(key_value_arr.length === 2) {
		    params[key_value_arr[0]] = key_value_arr[1];
		}
		else {
		    params[key_value_arr[0]] = "";
		}
	    }
	}
    }
}

var subscribe_video = (params.subscribe_video && params.subscribe_video.length > 0) ? params.subscribe_video : null;
var publish_video = (params.publish_video && params.publish_video.length > 0) ? params.publish_video : null;

var subscribe_audio = (params.subscribe_audio && params.subscribe_audio.length > 0) ? params.subscribe_audio : null;
var publish_audio = (params.publish_audio && params.publish_audio.length > 0) ? params.publish_audio : null;

var connection = WebrtcRos.createConnection();
connection.onConfigurationNeeded = function() {
    var config_box = document.getElementById("config-box");

    config_box.innerHTML = "";
    if(subscribe_video) {
	config_box.innerHTML += "Subscribed Video: <b>" + subscribe_video + "</b>";
    }
    else {
	config_box.innerHTML += "Not Subscribed to Video";
    }
    config_box.innerHTML += "<br>";

    if(publish_video) {
	config_box.innerHTML += "Published Video: <b>" + publish_video + "</b>";
    }
    else {
	config_box.innerHTML += "Not Publishing Video";
    }
    config_box.innerHTML += "<br>";

    if(subscribe_audio) {
	config_box.innerHTML += "Subscribed Audio: <b>" + subscribe_audio + "</b>";
    }
    else {
	config_box.innerHTML += "Not Subscribed to Audio";
    }
    config_box.innerHTML += "<br>";

    if(publish_audio) {
	config_box.innerHTML += "Published Audio: <b>" + publish_audio + "</b>";
    }
    else {
	config_box.innerHTML += "Not Publishing Audio";
    }
    config_box.innerHTML += "<br>";


    if(subscribe_video || subscribe_audio) {
	var remote_stream_config = {};
	if(subscribe_video) {
	    remote_stream_config.video = {
		"id": "subscribed_video",
		"src": subscribe_video
	    };
	}
	if(subscribe_audio) {
	    remote_stream_config.audio = {
		"id": "subscribed_audio",
		"src": subscribe_audio
	    };
	}
	connection.addRemoteStream(remote_stream_config).then(function(event) {
	    console.log("Remote stream added", event, event.stream.getVideoTracks(), event.stream.getAudioTracks());
	    var remoteVideoElement = document.getElementById("remote-video");
	    remoteVideoElement.srcObject = event.stream;
	    event.remove.then(function(event){
		console.log("Remote stream removed", event);
		remoteVideoElement.srcObject = null;
	    });
	    window.remotestream = event.stream;
	});
    }

    if(publish_video || publish_audio) {
	var user_media_config = {};
	var local_stream_config = {};
	if(publish_video) {
	    user_media_config.video = true;
	    local_stream_config.video = {
		"dest": publish_video
	    };
	}
	if(publish_audio) {
	    user_media_config.audio = true;
	}

	connection.addLocalStream(user_media_config, local_stream_config).then(function(event) {
	    console.log("Local stream added", event, event.stream.getVideoTracks(), event.stream.getAudioTracks());
	    var localVideoElement = document.getElementById("local-video");
	    localVideoElement.srcObject = event.stream;
	    event.remove.then(function(event){
		console.log("Local stream removed", event);
		localVideoElement.srcObject = null;
	    });
	    window.localstream = event.stream;
	});
    }
    connection.sendConfigure();
};
connection.connect();
