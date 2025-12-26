FROM fbe-dockerreg.rwu.de/doz-iki/staehle-vls/amr-tb3:latest
RUN apt-get update && \
    apt-get install -y ros-humble-image-proc && \
    rm -rf /var/lib/apt/lists/*
