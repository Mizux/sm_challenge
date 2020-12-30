# Create a virtual environment with all tools installed
# ref: https://hub.docker.com/_/debian
FROM debian:latest AS env
LABEL maintainer="mizux.dev@gmail.com"
# Install system build dependencies
ENV PATH=$PATH:/usr/local/bin
RUN apt-get update -q \
&& apt-get install -yq git wget build-essential \
xorg-dev libgl1-mesa-dev \
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install CMake 3.18.5
RUN wget "https://cmake.org/files/v3.18/cmake-3.18.5-Linux-x86_64.sh" \
&& chmod a+x cmake-3.18.5-Linux-x86_64.sh \
&& ./cmake-3.18.5-Linux-x86_64.sh --prefix=/usr/local/ --skip-license \
&& rm cmake-3.18.5-Linux-x86_64.sh
RUN cmake -version

# Add the src to our build env
FROM env AS devel
WORKDIR /home/project
COPY . .

FROM devel AS build
RUN cmake -S. -Bbuild
RUN cmake --build build --target all -v
RUN cmake --build build --target install -v

FROM build AS test
RUN cmake --build build --target test

# Test install rules
FROM env AS install_env
COPY --from=build /usr/local /usr/local/

FROM install_env AS install_devel
WORKDIR /home/sample
COPY ci/sample .

FROM install_devel AS install_build
RUN cmake -S. -Bbuild
RUN cmake --build build --target all -v

FROM install_build AS install_test
RUN cmake --build build --target test
