FROM alpine

#RUN apk add --no-cache --update
RUN apk add \
    git \
    make

# RUN git clone --depth 1 https://github.com/lii-enac/djnn-cpp
# # FIXME src/utils/properties.cpp: std::replace_if std::unique
# RUN (cd djnn-cpp/src/utils; echo "#include <algorithm>" > /tmp/foo.cpp ; cat properties.cpp >> /tmp/foo.cpp; cp /tmp/foo.cpp properties.cpp; )

# RUN git clone --depth 1 https://github.com/lii-enac/smala

VOLUME /djnn-cpp
VOLUME /smala
VOLUME /app

# TODO..:
#RUN cd djnn-cpp; make install-pkgdeps; cd ..
#RUN cd smala; make install-pkgdeps; de ..

# # ...meanwhile:
RUN apk add \
    g++ \
    pkgconfig \
    bison \
    flex \
    flex-dev \
    expat-dev \
    curl-dev \
    eudev \
    eudev-dev

# # not on armv7 ??
# # requires community in /etc/apk/repositories
RUN echo "http://dl-cdn.alpinelinux.org/alpine/v3.22/community" >> /etc/apk/repositories
RUN apk add \
    qt5-qtbase-dev \
    qt5-qt3d-dev \
    libevdev-dev \
    openal-soft-dev


#RUN cd /djnn-cpp; pwd; ls; make -j8;  cd ..
#RUN cd /smala && make -j8 lib; cd ..
#RUN cd /smala; ls; make -j simplest



#VOLUME /build
#WORKDIR /build

#CMD ["ash"]


