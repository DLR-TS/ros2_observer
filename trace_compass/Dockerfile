FROM openjdk:17.0.1-jdk-slim

RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgtk-3-0 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libasound2 \
    x11-xserver-utils \
    openjfx \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN wget https://eclipse.mirror.wearetriple.com//tracecompass/releases/10.2.0/rcp/trace-compass-10.2.0-20241204-1911-linux.gtk.x86_64.tar.gz
RUN tar -xvf *.tar.gz

WORKDIR /opt/trace-compass
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh

CMD ["./entrypoint.sh"]

