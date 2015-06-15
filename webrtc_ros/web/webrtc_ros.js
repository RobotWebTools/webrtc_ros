window.RTCPeerConnection = window.mozRTCPeerConnection || window.webkitRTCPeerConnection;
window.RTCSessionDescription = window.mozRTCSessionDescription || window.RTCSessionDescription;
window.RTCIceCandidate = window.mozRTCIceCandidate || window.RTCIceCandidate;
window.navigator.getUserMedia = navigator.mozGetUserMedia || navigator.webkitGetUserMedia;
window.URL = window.webkitURL || window.URL;

window.WebrtcRos = (function() {
    var WebrtcRosConnection = function(signalingServerPath) {
	var self = this;
	signalingServerPath = signalingServerPath || "ws://"+window.location.host+"/webrtc";
	this.signalingChannel = new WebSocket(signalingServerPath);
	this.signalingChannel.onmessage = function(e){ self.onSignalingMessage(e); };
	this.peerConnection = null;
	this.config = {};

	this.peerConnectionOptions = {
	    optional: [
		{DtlsSrtpKeyAgreement: true}
	    ]
	};
	this.connect();
    };

    Object.defineProperty(WebrtcRosConnection.prototype, "onSignalingOpen", {
	enumerable: true,
	get: function() { return this.signalingChannel.onopen; },
	set: function(callback) { this.signalingChannel.onopen = callback; }
    });
    Object.defineProperty(WebrtcRosConnection.prototype, "onSignalingError", {
	enumerable: true,
	get: function() { return this.signalingChannel.onerror; },
	set: function(callback) { this.signalingChannel.onerror = callback; }
    });
    Object.defineProperty(WebrtcRosConnection.prototype, "onSignalingClose", {
	enumerable: true,
	get: function() { return this.signalingChannel.onclose; },
	set: function(callback) { this.signalingChannel.onclose = callback; }
    });

    Object.defineProperty(WebrtcRosConnection.prototype, "onRemoteStreamAdded", {
	enumerable: true,
	get: function() { return this.peerConnection.onaddstream; },
	set: function(callback) { this.peerConnection.onaddstream = callback; }
    });
    Object.defineProperty(WebrtcRosConnection.prototype, "onRemoteStreamRemoved", {
	enumerable: true,
	get: function() { return this.peerConnection.onremovestream; },
	set: function(callback) { this.peerConnection.onremovestream = callback; }
    });
    WebrtcRosConnection.prototype.connect = function(){
	var self = this;
	this.peerConnection = new RTCPeerConnection(null, this.peerConnectionOptions);
	this.peerConnection.onicecandidate = function(event) {
            if (event.candidate) {
                var candidate = {
                    sdp_mline_index: event.candidate.sdpMLineIndex,
                    sdp_mid: event.candidate.sdpMid,
                    candidate: event.candidate.candidate,
                    type: "ice_candidate"
                };
                self.signalingChannel.send(JSON.stringify(candidate));
            }
        };
	this.peerConnection.onconnecting = this.onSessionConnecting;
        this.peerConnection.onopen = this.onSessionOpened;
    };
    WebrtcRosConnection.prototype.configure = function(config){
	this.config = config;
	var configMessage = {"type": "configure"};
	for(var field in config){
	    if(config.hasOwnProperty(field)) {
		configMessage[field] = config[field];
	    }
	}
	this.signalingChannel.send(JSON.stringify(configMessage));
	console.log("WebRTC ROS Configure: ", config);
    };
    WebrtcRosConnection.prototype.onSignalingMessage = function(e){
	var self = this;
	var dataJson = JSON.parse(e.data);
	console.log("WebRTC ROS Got Message: ", dataJson);
	if (dataJson.type === "offer") {
	    this.peerConnection.setRemoteDescription(new RTCSessionDescription(dataJson), this.onRemoteSdpSucces, this.onRemoteSdpError);
	    var sendVideo = this.config.published_video_topic && this.config.published_video_topic.length > 0;
	    if(sendVideo) {
		navigator.getUserMedia({"video": sendVideo}, function (stream) {
		    console.log("Video: ", stream.getVideoTracks());
		    self.peerConnection.addStream(stream);
		    self.sendAnswer();
		}, function(error) {
		    console.error("Error getting user media: ", error);
		});
	    }
	    else {
		self.sendAnswer();
	    }
	}
	else if(dataJson.type === "ice_candidate") {
	    var candidate = new RTCIceCandidate({sdpMLineIndex: dataJson.sdpMLineIndex, candidate: dataJson.candidate});
	    this.peerConnection.addIceCandidate(candidate);
	}
	else {
	    console.warn("Unknown message type: ", dataJson.type);
	}
    };

    WebrtcRosConnection.prototype.sendAnswer = function() {
	var self = this;
	var mediaConstraints = {"mandatory": {
	    "OfferToReceiveAudio": true,
	    "OfferToReceiveVideo": true
	}};
	this.peerConnection.createAnswer(function(sessionDescription) {
            console.log("Create answer:", sessionDescription);
            self.peerConnection.setLocalDescription(sessionDescription);
            var data = JSON.stringify(sessionDescription);
            self.signalingChannel.send(data);
	}, function(error) {
            console.log("Create answer error:", error);
	}, mediaConstraints);
    };

    WebrtcRosConnection.prototype.onSessionConnecting = function(message) {
        console.log("Session connecting: ", message);
    };

    WebrtcRosConnection.prototype.onSessionOpened = function(message) {
        console.log("Session opened: ", message);
    };

    WebrtcRosConnection.prototype.onRemoteSdpError = function(event) {
	console.error("onRemoteSdpError", event);
    };

    WebrtcRosConnection.prototype.onRemoteSdpSucces = function() {
	console.log("onRemoteSdpSucces");
    };

    var WebrtcRos = {
	createConnection: function(signalingServerPath) {
	    return new WebrtcRosConnection(signalingServerPath);
	}
    };
    return WebrtcRos;
})();
