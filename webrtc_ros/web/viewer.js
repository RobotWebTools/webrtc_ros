function on_ready(callback)
{
	if (document.readyState !== "loading")
		callback.call(document);
	else
		document.addEventListener("DOMContentLoaded", callback.bind(document));
}

on_ready(function() {
	let Q = {};
	let pv = window.location.search.substring(1).split("&");
	for (let i in pv)
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
		document.getElementById("topic").textContent = Q.subscribe_video;
		console.log("Establishing WebRTC connection");
		let conn = WebrtcRos.createConnection();
		conn.onConfigurationNeeded = function()
		{
			console.log("Requesting WebRTC video subscription");
			let config = {};
			config.video = {"id": "subscribed_video", "src": Q.subscribe_video};
			conn.addRemoteStream(config).then(function(event) {
				console.log("Connecting WebRTC stream to <video> element");
				document.getElementById("remote-video").srcObject = event.stream;
				event.remove.then(function(event) {
					console.log("Disconnecting WebRTC stream from <video> element");
					document.getElementById("remote-video").srcObject = null;
				});
			});
			conn.sendConfigure();
		}
		conn.connect();
	}
	else
	{
		document.getElementById("topic").textContent = "not connected";
		console.log("No subscribe_video parameter, not connecting to WebRTC video stream");
	}
});
