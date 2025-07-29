see https://github.com/kusharami/docker-qt5-alpine-dev

# Building

- Launch Docker (or OrbStack on macOS).
- Build an image

here 64-bit on a moderately recent machine

```
docker build --tag smala .
```

or (32-bit, for rpi2, beware, it's emulated on a 64-bit machine...)

```
docker build --platform linux/arm/v7 --tag smala_32 .
```

# Testing

- Launch XQuartz, then in a XQuartz xterm:
```
xhost +
```

- then in a macOS/Linux/WSL terminal:

```
docker run \
    -e DISPLAY=host.docker.internal:0 -v /tmp/.X11-unix:/tmp/.X11-unix \
    -u $(id -u):$(id -g) \
    -v /Users/conversy/recherche/istar/code/djnn/djnn-cpp:/djnn-cpp \
    -v /Users/conversy/recherche/istar/code/djnn/smala:/smala \
    -v /Users/conversy/recherche/helico-volta/code/volta:/app \
    -itd \
    --name smala \
    smala
```

//sh -c "cd smala && make -j simplest_test"


```
#docker exec -it smala sh -c "cd /djnn-cpp; make -j clear; cd /smala; make -j clear; cd /app; make; make -j clear;"
docker exec -it smala sh -c "cd /djnn-cpp; make -j; cd /smala; make -j lib; cd /app; make; make -j test;"

```


# Usage with host srcs TODO

```
#apk add expat curl eudev
apk add curl eudev udev-init-scripts
apk add qt5-qtbase qt5-qt3d libevdev openal-soft
```

mkdir res ...
export QT_QPA_PLATFORM=linuxfb
put fonts/* in /root/.fonts

-/boot/config.txt
/boot/usercfg.txt
dtparam=i2c_arm=on

/etc/modules
i2c-dev

/etc/udev/rules.d/10-local.rules
ACTION=="add", ATTRS{product}=="USBtin", KERNELS=="1-1.4", SYMLINK+="can_tritium"
ACTION=="add", ATTRS{product}=="USBtin", KERNELS=="1-1.5", SYMLINK+="can_arduino"

rc-update add udev sysinit
rc-update add udev-trigger sysinit
rc-update add udev-settle sysinit
rc-update add udev-postmount default

fix page_navigation_gpio


cd /app
/app/build/Linux-armv7l/volta

vi /etc/init.d/volta

#!/sbin/openrc-run
export QT_QPA_PLATFORM=linuxfb
export XDG_RUNTIME_DIR=/app

supervisor="supervise-daemon"
command=/app/build/Linux-armv7l/volta

start() {
        supervise-daemon volta --chdir /app --start /app/build/Linux-armv7l/volta
}

chmod +x /etc/init.d/volta

rc-update add volta default
rc-service volta start


# cross compiling (to run compiler with native-like performances)
# https://github.com/tonistiigi/xx
docker build --platform linux/arm/v7 --tag smala_xx_32 --progress=plain --no-cache -f Dockerfile-xx .

docker run \
    -e DISPLAY=host.docker.internal:0 -v /tmp/.X11-unix:/tmp/.X11-unix \
    -u $(id -u):$(id -g) \
    -v /Users/conversy/recherche/istar/code/djnn/djnn-cpp:/djnn-cpp \
    -v /Users/conversy/recherche/istar/code/djnn/smala:/smala \
    -v /Users/conversy/recherche/helico-volta/code/volta:/app \
    -it \
    --name smala_xx_32 \
    smala_xx_32

