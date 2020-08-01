FROM debian:buster-slim

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/London

# Install the essential packages
RUN mkdir -p /usr/share/man/man1
RUN apt-get update -y && apt-get install -y git autoconf build-essential gperf bison \
     flex texinfo libtool libncurses5-dev wget gawk \
     python-serial libexpat1-dev unzip \
     zip gdb make unrar-free automake gcc g++ libc6-dbg \
     libncurses-dev expat lua5.1 lua5.1-doc luarocks sed git bash \
     help2man bzip2 libtool-bin

RUN luarocks install luasocket && \
    luarocks install bitlib && \
    luarocks install luafilesystem && \ 
    luarocks install md5 && \
    luarocks install luaposix

RUN git clone --recurse-submodules -b 3.0-master_20190907 https://github.com/nodemcu/nodemcu-firmware.git
COPY nodemcu_firmware.patch nodemcu-firmware/
RUN cd nodemcu-firmware && \
    git apply --stat nodemcu_firmware.patch && \
    git apply --check nodemcu_firmware.patch && \
    make -j$((`nproc`-1)) && mv luac.cross /usr/bin/ && \
    mv bin /firmware && cd / && rm -rf nodemcu-firmware

RUN rm -rf /var/lib/apt/lists/*

RUN mkdir /in
WORKDIR /in

RUN echo "\
#!/bin/bash
luac.cross -f -o /out/lfs.img /in/*.lua \n\
cp -r /firmware/ /out/ \n\
"> /usr/bin/compile
RUN chmod +x /usr/bin/compile

ENTRYPOINT ["bash", "compile" ]
