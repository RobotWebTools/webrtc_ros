function onload() {
    var data_request = new XMLHttpRequest();
    data_request.onload = function()
    {
        if (data_request.status === 200)
        {
            var found = false;
            var stream_data = JSON.parse(data_request.responseText);
            var camera_topics_list = document.getElementById("camera_topics_list");
            for (var base_topic in stream_data.camera_topics)
            {
                if (stream_data.camera_topics.hasOwnProperty(base_topic))
                {
                    found = true;
                    var camera_li = document.createElement("li");
                    camera_li.appendChild(document.createTextNode(base_topic));
                    var camera_topic_list = document.createElement("ul");
                    for (var topic_name in stream_data.camera_topics[base_topic])
                    {
                        if(stream_data.camera_topics[base_topic].hasOwnProperty(topic_name))
                        {
                            var camera_topic_li = document.createElement("li");
                            var camera_topic_a = document.createElement("a");
                            camera_topic_a.appendChild(document.createTextNode(topic_name));
                            camera_topic_a.href = "/viewer?subscribe_video=ros_image:" + stream_data.camera_topics[base_topic][topic_name];
                            camera_topic_li.appendChild(camera_topic_a);
                            camera_topic_list.appendChild(camera_topic_li);
                        }
                    }
                    camera_li.appendChild(camera_topic_list);
                    camera_topics_list.appendChild(camera_li);
                }
            }
            var image_topic_list = document.getElementById("image_topics_list");
            for (var i in stream_data.image_topics)
            {
                if (stream_data.image_topics.hasOwnProperty(i))
                {
                    found = true;
                    var topic = stream_data.image_topics[i];
                    var topic_li = document.createElement("li");
                    var topic_a = document.createElement("a");
                    topic_a.appendChild(document.createTextNode(topic));
                    topic_a.href = "/viewer?subscribe_video=ros_image:" + topic;
                    topic_li.appendChild(topic_a);
                    image_topic_list.appendChild(topic_li);
                }
            }
            if (!found)
            {
                document.getElementById("status").appendChild(document.createTextNode("No topics found"));
            }
        }
        else
        {
            document.getElementById("status").appendChild(document.createTextNode("Query failed (HTTP status "+ data_request.status + ")"));
        }
    };
    data_request.open("GET", "/list_streams.json", true);
    data_request.send();
    document.getElementById("custom_topic_form").addEventListener("submit", function(event)
    {
        var form = event.target;
        var params = [];
        if (form.publish_video_topic.value.length > 0)
        {
            params.push("publish_video=ros_image:" + form.publish_video_topic.value);
        }
        if (form.subscribe_video_topic.value.length > 0)
        {
            params.push("subscribe_video=ros_image:" + form.subscribe_video_topic.value);
        }
        if (form.publish_local_audio.checked)
        {
            params.push("publish_audio=local:");
        }
        if(form.subscribe_local_audio.checked)
        {
            params.push("subscribe_audio=local:");
        }
        event.preventDefault();
        window.location = "/viewer?" + params.join("&");
    });
}
