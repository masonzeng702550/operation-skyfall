# ArduPilot SITL — headless, arm64/amd64 native build
# Pinned to a stable Copter branch so challenge behaviour is reproducible.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
# Pin the exact release. Q2 (mission-parse SIGFPE) depends on an unpatched
# tree — never build this from a moving branch.
ARG ARDUPILOT_BRANCH=Copter-4.6.3
ENV ARDUPILOT_BRANCH=${ARDUPILOT_BRANCH}

RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates \
        python3 python3-pip python3-dev python3-setuptools \
        g++ ccache rsync procps tini \
        libtool libxml2-dev libxslt1-dev zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

# empy MUST be 3.3.x — ArduPilot's waf breaks on empy 4.x
RUN pip3 install --no-cache-dir \
        "empy==3.3.4" future lxml pexpect packaging \
        pymavlink MAVProxy

WORKDIR /opt
RUN git clone --depth 1 -b ${ARDUPILOT_BRANCH} https://github.com/ArduPilot/ardupilot.git

WORKDIR /opt/ardupilot
RUN git submodule update --init --recursive --depth 1

RUN ./waf configure --board sitl && ./waf copter && ccache -s

COPY start.sh /opt/start.sh
COPY mission.txt /opt/mission.txt
COPY challenge.parm /opt/challenge.parm
RUN chmod +x /opt/start.sh

# 14550/udp -> competitor entry point   14551/udp -> read-only feed to watcher
EXPOSE 14550/udp

ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["/opt/start.sh"]
