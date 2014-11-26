window.WebrtcRos = (function() {
    var WebrtcRosConnection = function(signalingServerPath) {
	var self = this;
	signalingServerPath = signalingServerPath || 'ws://'+window.location.host+'/webrtc';
	this.signalingChannel = new WebSocket(signalingServerPath);
	this.signalingChannel.onmessage = function(e){ self.onSignalingMessage(e); };
	this.peerConnection = null;
	this.config = {};

	var peerConnectionOptions = {
	    optional: [
		{DtlsSrtpKeyAgreement: true}
	    ]
	};
	this.peerConnection = new RTCPeerConnection(null, this.peerConnectionOptions);
	this.peerConnection.onicecandidate = function(event) {
            if (event.candidate) {
                var candidate = {
                    sdp_mline_index: event.candidate.sdpMLineIndex,
                    sdp_mid: event.candidate.sdpMid,
                    candidate: event.candidate.candidate,
                    type: 'ice_candidate'
                };
		console.log(self);
                self.signalingChannel.send(JSON.stringify(candidate));
            } else {
                //console.log("End of candidates.");
            }
        };
	this.peerConnection.onconnecting = this.onSessionConnecting;
        this.peerConnection.onopen = this.onSessionOpened;
        this.peerConnection.onaddstream = this.onRemoteStreamAdded;
        this.peerConnection.onremovestream = this.onRemoteStreamRemoved;
    };

    Object.defineProperty(WebrtcRosConnection.prototype, 'onSignalingOpen', {
	get: function() { return this.signalingChannel.onopen; },
	set: function(callback) { this.signalingChannel.onopen = callback; }
    });
    Object.defineProperty(WebrtcRosConnection.prototype, 'onSignalingError', {
	get: function() { return this.signalingChannel.onerror; },
	set: function(callback) { this.signalingChannel.onerror = callback; }
    });
    WebrtcRosConnection.prototype.configure = function(config){
	this.config = config;
	var configMessage = {"type": "configure"};
	for(var field in config){
	    configMessage[field] = config[field];
	}
	this.signalingChannel.send(JSON.stringify(configMessage));
	console.log("WebRTC ROS Configure: ", config);
    };
    WebrtcRosConnection.prototype.onSignalingMessage = function(e){
	var self = this;
	var dataJson = JSON.parse(e.data);
	console.log("WebRTC ROS Got Message: ", dataJson);
	if (dataJson['type'] == 'offer') {
	    this.peerConnection.setRemoteDescription(new RTCSessionDescription(dataJson), this.onRemoteSdpSucces, this.onRemoteSdpError);

	    if(this.config["published_video_topic"] && this.config["published_video_topic"].length > 0) {
		navigator.getUserMedia({"video": true}, function (stream) {
		    self.peerConnection.addStream(stream);
		    self.sendAnswer();
		}, function(error) {
		    console.error(error);
		});
	    }
	    else
		self.sendAnswer();
	}
	else if(dataJson['type'] == 'ice_candidate') {
	    var candidate = new RTCIceCandidate({sdpMLineIndex: dataJson.sdpMLineIndex, candidate: dataJson.candidate});
	    this.peerConnection.addIceCandidate(candidate);
	}
	else {
	    console.warn("Unknown message type: ", dataJson['type']);
	}
    };

    WebrtcRosConnection.prototype.sendAnswer = function() {
	var self = this;
	var mediaConstraints = {'mandatory': {
	    'OfferToReceiveAudio': true,
	    'OfferToReceiveVideo': true
	}};
	this.peerConnection.createAnswer(function(sessionDescription) {
            console.log("Create answer:", sessionDescription);
            self.peerConnection.setLocalDescription(sessionDescription);
            var data = JSON.stringify(sessionDescription);
            self.signalingChannel.send(data);
	}, function(error) {
            console.log("Create answer error:", error);
	}, mediaConstraints);
    }

    WebrtcRosConnection.prototype.onRemoteStreamAdded = function(event) {
	console.log(event);
	console.log("Remote stream added:", URL.createObjectURL(event.stream));
	var remoteVideoElement = document.getElementById('remote-video');
	remoteVideoElement.src = URL.createObjectURL(event.stream);
	remoteVideoElement.play();
    }
    WebrtcRosConnection.prototype.onSessionConnecting = function(message) {
        console.log("Session connecting.");
    }

    WebrtcRosConnection.prototype.onSessionOpened = function(message) {
        console.log("Session opened.");
    }

    WebrtcRosConnection.prototype.onRemoteStreamRemoved = function(event) {
        console.log("Remote stream removed.");
    }
    WebrtcRosConnection.prototype.onRemoteSdpError = function(event) {
	console.error('onRemoteSdpError', event);
	console.error('onRemoteSdpError', event.name, event.message);
    }

    WebrtcRosConnection.prototype.onRemoteSdpSucces = function() {
	console.log('onRemoteSdpSucces');
    }

    var WebrtcRos = {
	createConnection: function(signalingServerPath) {
	    return new WebrtcRosConnection(signalingServerPath);
	}
    };
    return WebrtcRos;
})();
