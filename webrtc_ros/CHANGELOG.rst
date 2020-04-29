^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webrtc_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add ICE to server (`#44 <https://github.com/RobotWebTools/webrtc_ros/issues/44>`_)
* Implement logic to resize or drop frames on demand
* Get rid of jQuery
* Ignore invalid ICE candidates
  This resolves an issue with Firefox.
* Update JavaScript client code for latest browser compatibility
* Contributors: Michael Sobrepera, Timo Röhling

59.0.3 (2019-01-25)
-------------------
* No changes

59.0.2 (2019-01-24)
-------------------
* Do not install files outside the package source folder
  I of all people should have caught that.
* Contributors: Timo Röhling

59.0.1 (2019-01-24)
-------------------
* No changes

59.0.0 (2019-01-18)
-------------------
* Upgrade server implementation to support the new API 59
* Remove deprecated browser features from JS code
* Only subscribe once to each ROS image topic, regardless of the number of WebRTC clients
* Various other fixes
* Contributors: Aaron Gokaslan, Mitchell Wills, Robot User, Russell Toris, Timo Röhling

