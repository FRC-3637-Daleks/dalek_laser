FROM ubuntu:jammy

# Install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl \
        cpufrequtils \
        libatomic1 \
        openjdk-11-jdk \
        libcholmod3 \
        liblapack3 \
        libsuitesparseconfig5 \
        v4l-utils \
        wget \
        sqlite3 && \
    rm -rf /var/lib/apt/lists/*


RUN mkdir -p /opt/photonvision
WORKDIR /opt/photonvision

COPY get_arch_name.sh .

# Find architecture and download correct version of photonvision jar
RUN ARCH_NAME="$(sh get_arch_name.sh)" && \
    echo "Downloading photonvision for $ARCH_NAME" && \
    curl -sk https://api.github.com/repos/photonvision/photonvision/releases/latest | \
        grep "browser_download_url.*$ARCH_NAME.jar" | \
        cut -d : -f 2,3 | \
        tr -d '"' | \
        wget -qi - -O photonvision.jar

VOLUME /opt/photonvision/photonvision_config

CMD ["/usr/bin/java", "-Xmx512m", "-jar", "/opt/photonvision/photonvision.jar"]