# webrtc_ros Signaling Protocol Specification

This document outlines the signaling protocol used by the webrtc_ros package to
communicate between clients. It was initially designed to communicate between a
web client and webrtc_ros_server using Websockets, but has the potential to
connect two ROS nodes so that it could be used as an image transport.

While WebRTC provides the required infrastructure to stream video, audio, and
data in real time, it requires a signaling channel to be established in order
to exchange information in order to create the WebRTC connection. Additionally,
the signaling channel is used to exchange information for which ROS topics to
use. Usually the signaling channel is setup by a server that two clients
connect to, but in order to make webrtc_ros_server self contained it acts as
a server and a WebRTC client. See more information
[here](http://www.html5rocks.com/en/tutorials/webrtc/infrastructure/).

The webrtc_ros signaling protocol is based on an exchange of JSON objects.
Currently each object is sent in it's own Websocket message in order to
easily separate them. Each message contains a type field that distinguishes
between different message types.

## 1. The webrtc_ros signaling transport

webrtc_ros_server accepts Websocket connections (by default on port 8080 at
"/webrtc"). Each Websocket connection will create a corresponding WebRTC client
on the server. However, the peer connections will not begin exchanging SDP
messages until the first configure message is sent. Messages take the basic
form of:

```json
{ "type": <string> }
```

Additional fields can also be specified to send additional information.

## 2. The webrtc_ros signaling protocol

Message Types:
 * **ice_candidate** - A message that contains an ICE candidate
 * **offer** - A message that contains a WebRTC SDP offer
 * **answer** - A message that contains a WebRTC SDP answer
 * **configure** - A message that configures media and data streams


### 2.1 ICE Candidate Message

ICE candidate messages are sent with ICE candidates that are used by WebRTC to
establish connections. Messages take the form:

```json
{ "type": "ice_candidate",
  "sdp_mid": <string>,
  "sdp_mline_index": <int>,
  "candiate": <string>
}
```

More documentation on the attributes can be found
[here](http://www.w3.org/TR/webrtc/#attributes-3).


### 2.2 SDP Offer and Answer Message

Offer and Answer messages are exchanged by WebRTC to describe the capabilities
and streams of the clients. They take the form:

```json
{ "type": "offer" | "answer",
  "sdp": <string>
}
```

### 2.3 Configure Message

Configure messages allow the clients to request actions of the other client.
They can be used to ask a client to add a stream that is a republishing of a
ROS topic, remove a stream, etc. Once a configure message is sent the receiver
responds with an SDP offer, which is responded to with a SDP answer. Configure
messages take the form:

```json
{ "type": "configure",
  "actions": [<action>]
}
```

An action is of the form:
```json
{ "type": <string> }
```
with additional properties depending on the type of the action.

Action Types:
 * **add_stream** - Tell the remote client start a new stream
 * **remove_stream** - Tell the remote client to remove a stream
 * **add_video_track** - Add a video track to a remote client's stream
 * **add_audio_track** - Add a audio track to a remote client's stream
 * **expect_stream** - Tell the remote client to expect a stream
 * **expect_video_track** - Tell the remote client to expect a video track
    and what to do with the track it receives

### 2.3.1 add_stream

```json
{ "type": "add_stream",
  "id": <string>
}
```

### 2.3.2 remove_stream

```json
{ "type": "remove_stream",
  "id": <string>
}
```

### 2.3.3 add_video_track

```json
{ "type": "add_video_track",
  "stream_id": <string>,
  "id": <string>,
  "src": <string>
}
```

### 2.3.3 add_audio_track

```json
{ "type": "add_audio_track",
  "stream_id": <string>,
  "id": <string>,
  "src": <string>
}
```

When adding the stream the remote client uses the src field to determine where
to get the video from.

### 2.3.4 expect_stream

```json
{ "type": "expect_stream",
  "id": <string>
}
```

### 2.3.5 expect_video_track

```json
{ "type": "expect_video_track",
  "stream_id": <string>,
  "id": <string>,
  "dest": <string>
}
```

### 3. Stream Sources
Stream sources are specified as a URI with a scheme and path component.

### 3.1 Video Track Sources

### 3.1.1 ROS Image Source ( ros\_image:_ros\_topic_ )
The ROS image source uses image_transport to subscribe to images from a ROS
system. It uses the received images as the video track. The URI path component
is used as the subscribed ROS topic.

### 3.2 Audio Track Sources ( local: )
Streams audio from inputs on the machine the webrtc_ros node is running on.
Currently this just uses the default input source, but may be expanded in the
future to add support for selecting a source.

### 3.2.1 Local Audio Source
The ROS image source uses image_transport to subscribe to images from a ROS
system. It uses the received images as the video track. The URI path component
is used as the subscribed ROS topic.


### 4. Stream Destinations
Stream destinations are specified as a URI with a scheme and path component.

### 4.1 Video Track Destinations

### 4.1.1 ROS Image Destinations ( ros\_image:_ros\_topic_ )
The ROS image destination uses image_transport to publish to images from a ROS
system. It publishes the received frames to the ROS topic represented by the
URI path component.

### 4.2 Audio Track Destinations

Currently it is not possible to select the destination of an audio track. All
received audio tracks will be output on the default audio device on the machine
the server is running on.