$(document).ready(function() {
	$("#status").empty().text("Looking for available streams...");
	$.getJSON("list_streams.json", function(json) {
		for (var base_topic in json.camera_topics)
		{
			if (json.camera_topics.hasOwnProperty(base_topic))
			{
				var camera_li = $("<li>");
				camera_li.text(base_topic);
				var topic_ul = $("<ul>");
				for (var topic_name in json.camera_topics[base_topic])
				{
					if (json.camera_topics[base_topic].hasOwnProperty(topic_name))
					{
						topic_ul.append(
							$("<li>").append(
								$("<a>").attr("href", "viewer?subscribe_video=ros_image:" + json.camera_topics[base_topic][topic_name]).text(topic_name)
							)
						);
					}
				}
				camera_li.append(topic_ul);
				$("#camera_topics_list").append(camera_li);
			}
		}
		for (var i in json.image_topics)
		{
			if (json.image_topics.hasOwnProperty(i))
			{
				$("#image_topics_list").append(
					$("<li>").append(
						$("<a>").attr("href", "viewer?subscribe_video=ros_image:" + json.image_topics[i]).text(json.image_topics[i])
					)
				);
			}
		}
		$("#status").empty();
	});
});

