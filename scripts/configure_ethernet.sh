#!/bin/bash

# Define variables
CONNECTION_NAME="igus_robotic_arm"
INTERFACE_NAME="enP8p1s0"
STATIC_IP="192.168.3.10/24"
GATEWAY="192.168.3.1"
DNS_SERVERS="8.8.8.8,8.8.4.4"

# Check if the connection already exists
if nmcli con show "$CONNECTION_NAME" &>/dev/null; then
    echo "Connection '$CONNECTION_NAME' already exists. Modifying existing connection."
    nmcli con mod "$CONNECTION_NAME" \
    ipv4.addresses "$STATIC_IP" \
    ipv4.gateway "$GATEWAY" \
    ipv4.dns "$DNS_SERVERS" \
    ipv4.method manual
else
    echo "Creating new connection '$CONNECTION_NAME'."
    nmcli con add type ethernet con-name "$CONNECTION_NAME" ifname "$INTERFACE_NAME" \
    ipv4.addresses "$STATIC_IP" \
    ipv4.gateway "$GATEWAY" \
    ipv4.dns "$DNS_SERVERS" \
    ipv4.method manual
fi

# Bring the connection up
nmcli con up "$CONNECTION_NAME"

echo "Configuration applied. Connection '$CONNECTION_NAME' is now active with IP $STATIC_IP."
