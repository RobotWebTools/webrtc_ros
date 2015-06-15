var build_tasks = ["jshint"];
module.exports = function(grunt) {
    "use strict";
    grunt.initConfig({
	pkg: grunt.file.readJSON("package.json"),
	jshint: {
	    options: {
		browser: true,
		devel: true,
		globals: {
		    module: false,
		    JSON: false,
		    RTCPeerConnection: true,
		    RTCSessionDescription: true,
		    RTCIceCandidate: true
		},

		enforceall: true,
		camelcase: false,
		nocomma: false,
		strict: false,
		singleGroups: false,

		quotmark: "double",
		maxparams: 6,
		maxdepth: 3,
		maxstatements: 30,
		maxcomplexity: 6
	    },
	    files: ["Gruntfile.js", "*.js"]
	}
    });

    grunt.loadNpmTasks("grunt-contrib-jshint");
    grunt.registerTask("default", ["jshint"]);
};
