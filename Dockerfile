FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    curl ca-certificates build-essential pkg-config \
    libx11-dev libxcb1-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev \
    libxkbcommon-dev libwayland-dev libssl-dev \
    mesa-vulkan-drivers vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
ENV PATH=/root/.cargo/bin:$PATH

WORKDIR /work
COPY . .
RUN cargo build --release

ENTRYPOINT ["./target/release/app"]
CMD ["--mock"]
