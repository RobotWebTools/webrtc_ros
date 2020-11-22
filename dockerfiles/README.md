# Dockerfiles

These dockerfiles are useful for testing the webrtc ros package.

To run them, you will need to:
1. Install Docker: (official docs)[https://docs.docker.com/v17.09/engine/installation/linux/docker-ce/ubuntu/] or (simplified)[https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-18-04]
2. Build the images: `bash build_images.sh`
3. Run the image: `docker run -p 8080:8080 --name <container name> webrtc_ros/<dockerfile>`
4. Open your web-browser and go to http://localhost:8080/
5. Kill the container when done with `docker kill <container name>`

Notes:
- `container name` can be whatever you want
- `dockerfile` should come from the list below
- you could choose to not give the docker container an image when running. You would then want to get the name with `docker container ls` when you are ready to kill it.
- if you want to run with an interactive terminal, use: `docker run -p 8080:8080 -it webrtc_ros/<dockerfile>` (the difference is the `-it` for interactive terminal mode) and then to kill, just press ctrl-c

## Available Files:
These are the dockerfiles available
- released: this image is the version released on the package repository
- develop: this image is the version in the github develop branch
- dev_awsc14: This image is the develop branch and a (fix to the apache web server)[https://github.com/GT-RAIL/async_web_server_cpp/pull/14]
