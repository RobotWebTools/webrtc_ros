window.WebrtcRos = (function() {
    var WebrtcRosConnection = function(signalingServerPath) {
	this.signalingServerPath = signalingServerPath || "ws://"+window.location.host+"/webrtc";
	this.onConfigurationNeeded = undefined;
	this.signalingChannel = null;
	this.peerConnection = null;
	this.peerConnectionOptions = {
	    optional: [
		{DtlsSrtpKeyAgreement: true}
	    ]
	};
    };


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
	close();
	this.signalingChannel = new WebSocket(this.signalingServerPath);
	this.signalingChannel.onmessage = function(e){ self.onSignalingMessage(e); };
	this.signalingChannel.onopen = function() {
	    console.log("WebRTC signaling open");
	    if(self.onConfigurationNeeded) {
		self.onConfigurationNeeded();
	    }
	};
	this.signalingChannel.onerror = function() {
	    console.log("WebRTC signaling error");
	};
	this.signalingChannel.onclose = function() {
	    console.log("WebRTC signaling close");
	};
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
    };
    WebrtcRosConnection.prototype.close = function(){
	if(!this.peerConnection) {
	    this.peerConnection.close();
	    this.peerConnection = null;
	}
	if(!this.signalingChannel) {
	    this.signalingChannel.close();
	    this.signalingChannel = null;
	}
    };
    WebrtcRosConnection.prototype.configureBuilder = function(){
	return new WebrtcRosConfigureBuilder(this);
    };
    WebrtcRosConnection.prototype.sendConfigure = function(actions){
	var configMessage = {"type": "configure", "actions": actions};
	this.signalingChannel.send(JSON.stringify(configMessage));
	console.log("WebRTC ROS Configure: ", actions);
    };
    WebrtcRosConnection.prototype.onSignalingMessage = function(e){
	var self = this;
	var dataJson = JSON.parse(e.data);
	console.log("WebRTC ROS Got Message: ", dataJson);
	if (dataJson.type === "offer") {
	    this.peerConnection.setRemoteDescription(new RTCSessionDescription(dataJson),
						     function(){
							 self.sendAnswer();
						     },
						     function(event) {
							 console.error("onRemoteSdpError", event);
						     });
	}
	else if(dataJson.type === "ice_candidate") {
	    var candidate = new RTCIceCandidate({sdpMLineIndex: dataJson.sdp_mline_index, candidate: dataJson.candidate});
	    this.peerConnection.addIceCandidate(candidate);
	}
	else {
	    console.warn("Unknown message type: ", dataJson.type);
	}
    };

    WebrtcRosConnection.prototype.sendAnswer = function() {
	var self = this;
	var mediaConstraints = {"optional": [
	    {"OfferToReceiveVideo": true},
	    {"OfferToReceiveVideo": true}
	]};
	this.peerConnection.createAnswer(function(sessionDescription) {
            self.peerConnection.setLocalDescription(sessionDescription);
            var data = JSON.stringify(sessionDescription);
            self.signalingChannel.send(data);
	}, function(error) {
            console.warn("Create answer error:", error);
	}, mediaConstraints);
    };

    var WebrtcRosConfigureBuilder = function(connection) {
	this.connection = connection;
	this.actions = [];
	this.lastActionPromise = Promise.resolve();
    };
    WebrtcRosConfigureBuilder.prototype.addRemoteStream = function(stream_id, video_id, video_src) {
	var self = this;
	this.lastActionPromise = this.lastActionPromise.then(function() {
	    self.actions.push({"type":"add_stream", "id": stream_id});
	    if(video_id && video_src) {
		self.actions.push({
		    "type":"add_video_track",
		    "stream_id": stream_id,
		    "id": video_id,
		    "src": video_src
		});
	    }
	});
	return this;
    };
    WebrtcRosConfigureBuilder.prototype.addRemoteVideoTrack = function(stream_id, id, src) {
	var self = this;
	this.lastActionPromise = this.lastActionPromise.then(function() {
	});
	return this;
    };
    WebrtcRosConfigureBuilder.prototype.addLocalStream = function(configuration, videoDest) {
	var self = this;
	this.lastActionPromise = this.lastActionPromise.then(function(){return navigator.mediaDevices.getUserMedia(configuration).then(function(stream){
            self.actions.push({"type":"expect_stream", "id": stream.id});
	    if(videoDest) {
		self.actions.push({
		    "type":"expect_video_track",
		    "stream_id": stream.id,
		    "id": stream.getVideoTracks()[0].id,
		    "dest":videoDest
		});
	    }
            connection.peerConnection.addStream(stream);
	});});
	return this;
    };
    WebrtcRosConfigureBuilder.prototype.send = function() {
	var self = this;
	this.lastActionPromise.then(function() {
	    self.connection.sendConfigure(self.actions);
	});
    };

    var WebrtcRos = {
	createConnection: function(signalingServerPath) {
	    return new WebrtcRosConnection(signalingServerPath);
	}
    };
    return WebrtcRos;
})();
