FROM debian:bookworm-slim

# Picamera2 is distributed through Raspberry Pi's official APT archive.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    && curl -fsSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key \
        | gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] https://archive.raspberrypi.com/debian/ bookworm main" \
        > /etc/apt/sources.list.d/raspberrypi.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    i2c-tools \
    python3-gpiozero \
    python3-flask \
    python3-picamera2 \
    python3-pip \
    python3-rpi.gpio \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# The remaining dependencies are pure-Python CircuitPython libraries.
COPY requirements.txt .
RUN pip3 install --break-system-packages --no-cache-dir -r requirements.txt

# Copy application code
COPY app/ ./app/
COPY config/ ./config/

# Create captures directory
RUN mkdir -p /app/captures

CMD ["python3", "-m", "app.entrypoint"]
