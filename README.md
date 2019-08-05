# ros_duckie

```bash
git clone repo 

cd repo

docker build -t jdrew1303/duckie:master .

docker run -it --net host --memory="800m" --memory-swap="1.8g" --privileged --name rosbot -v /opt/vc/lib:/opt/vc/lib -v /mnt/usb:/opt/usb jdrew1303/duckie:master
```
