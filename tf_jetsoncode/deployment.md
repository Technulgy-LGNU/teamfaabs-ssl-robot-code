# Deployment

Jetson specific setup for realtime scheduling.

### 1 - Create a new user group, add your user and set permissions

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER

# Set permissions for the group
sudo vim /etc/security/limits.d/99-realtime.conf

# Contents of 99-realtime.conf
@realtime   soft rtprio     99
@realtime   hard rtprio     99

@realtime   soft memlock    unlimited
@realtime   hard memlock    unlimited

# Enable RT in Jetson Kernel
sudo vim /etc/sysctl.d/99-realtime.conf

# Contents of 99-realtime.conf
kernel.sched_rt_runtime_us = -1

# Reboot for changes to take effect
sudo reboot
```

### 2 - Create a systemd service for your application

```bash
sudo vim /etc/systemd/system/tf_jetson.service
```
With following content:
```ini
[Unit]
Description=tf_jetsoncode Robot Service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple

User=robotik
Group=robotik

WorkingDirectory=/home/robotik/tf_jetsoncode
ExecStart=/home/robotik/tf_jetsoncode/tf_jetsoncode

Restart=always
RestartSec=1

#################################################
# Realtime
#################################################

CPUSchedulingPolicy=fifo
CPUSchedulingPriority=80

LimitRTPRIO=99
LimitMEMLOCK=infinity

AmbientCapabilities=CAP_SYS_NICE
CapabilityBoundingSet=CAP_SYS_NICE

RestrictRealtime=no

#################################################
# Logging
#################################################

StandardOutput=journal
StandardError=journal
LogRateLimitIntervalSec=0

#################################################
# Shutdown
#################################################

KillSignal=SIGINT
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
```

### 3 - Enable and start the service

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now tf_jetson.service
```

### 4 - Check the status and logs

```bash
sudo systemctl status tf_jetson.service
sudo journalctl -u tf_jetson.service -f
```
