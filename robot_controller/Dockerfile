# syntax=docker/dockerfile:1

FROM alfredroberts/erebus

# Create a directory for my controller code to go
RUN apt-get update && apt-get install libsdl2-dev -y
RUN mkdir -p /usr/local/controller
WORKDIR /usr/local/controller
RUN mkdir -p build
# Copy my controller code from the dir my dockerfile is in to my
# current (work) directory
COPY . .
# RUN rm -R /usr/local/controller/build/*
# Compile my controller
WORKDIR /usr/local/controller/build
RUN cmake -DCOMPMODE=ON ..
RUN make -j$(nproc) && cp controller ../

WORKDIR /usr/local/controller

# Run my controller (using extern controller helper)
CMD webots-controller --protocol=tcp --ip-address=$EREBUS_SERVER --port=1234 --robot-name=Erebus_Bot --stdout-redirect --stderr-redirect controller
