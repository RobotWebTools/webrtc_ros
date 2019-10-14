$(document).ready(function() {
	var Q = {};
	var pv = window.location.search.substring(1).split("&");
	for (var i in pv)
	{
		if (pv.hasOwnProperty(i))
		{
			var kv = pv[i].split("=", 2);
			if (kv.length == 2)
			{
				Q[decodeURIComponent(kv[0])] = decodeURIComponent(kv[1]);
			}
		}
	}
	if (Q.subscribe_video)
	{
		$("#topic").text(Q.subscribe_video);
		console.log("Establishing WebRTC connection");
		var conn = WebrtcRos.createConnection();
		conn.onConfigurationNeeded = function()
		{
			console.log("Requesting WebRTC video subscription");
			var config = {};
			config.video = {"id": "subscribed_video", "src": Q.subscribe_video};
			conn.addRemoteStream(config).then(function(event) {
				console.log("Connecting WebRTC stream to <video> element");
				$("#remote-video").prop("srcObject", event.stream);
				event.remove.then(function(event) {
					console.log("Disconnecting WebRTC stream from <video> element");
					$("#remote-video").prop("srcObject", null);
				});
			});
			conn.sendConfigure();
		}
		conn.connect();
	}
	else
	{
		$("#topic").append("not connected");
		console.log("No subscribe_video parameter, not connecting to WebRTC video stream");
	}
});

