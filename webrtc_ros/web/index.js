function on_ready(callback)
{
	if (document.readyState !== "loading")
		callback.call(document);
	else
		document.addEventListener("DOMContentLoaded", callback.bind(document));
}

on_ready(function() {
	let status = document.getElementById("status");
	status.textContent = "Looking for available streams...";
	fetch("list_streams.json").then(data => data.json(), function(err) { status.textContent = err; }).then(function(json) {
		for (let base_topic in json.camera_topics)
		{
			if (json.camera_topics.hasOwnProperty(base_topic))
			{
				let camera_li = document.createElement("li");
				camera_li.textContent = base_topic;
				let topic_ul = document.createElement("ul");
				for (let topic_name in json.camera_topics[base_topic])
				{
					if (json.camera_topics[base_topic].hasOwnProperty(topic_name))
					{
						let topic_li = document.createElement("li");
						let topic_a = document.createElement("a");
						topic_a.href = "viewer?subscribe_video=ros_image:" + json.camera_topics[base_topic][topic_name];
						topic_a.textContent = topic_name;
						topic_li.appendChild(topic_a);
						topic_ul.appendChild(topic_li);
					}
				}
				camera_li.appendChild(topic_ul);
				document.getElementById("camera_topics_list").appendChild(camera_li);
			}
		}
		for (var i in json.image_topics)
		{
			if (json.image_topics.hasOwnProperty(i))
			{
				let topic_li = document.createElement("li");
				let topic_a = document.createElement("a");
				topic_a.href = "viewer?subscribe_video=ros_image:" + json.image_topics[i];
				topic_a.textContent = json.image_topics[i];
				topic_li.appendChild(topic_a);
				document.getElementById("image_topics_list").appendChild(topic_li);
			}
		}
		status.innerHTML = null;
	});
});
