# Create a virtual environment with all tools installed
# ref: https://hub.docker.com/_/fedora
FROM fedora:latest AS env
LABEL maintainer="mizux.dev@gmail.com"
# Install system build dependencies
ENV PATH=$PATH:/usr/local/bin
RUN dnf -y update \
&& dnf -y groupinstall "Development Tools" \
&& dnf -y install git cmake gcc-c++ \
libX11-devel mesa-libGL-devel \
libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel \
&& dnf clean all
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
