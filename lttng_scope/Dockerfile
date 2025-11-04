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

WORKDIR /opt/lttng-scope

RUN wget https://lttng.org/files/lttng-scope/lttng-scope-0.4.0-linux-x64-installer.run -O lttng-scope-installer.run

RUN chmod +x lttng-scope-installer.run

RUN ./lttng-scope-installer.run --mode unattended --unattendedmodeui none

ENV JAVA_OPTS="-Dprism.verbose=true"

WORKDIR /opt/lttng-scope/tracing_logs
CMD ["/opt/lttng-scope/lttng-scope.run"]

