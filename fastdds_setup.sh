#!/bin/bash
# fastdds_setup.sh
# Run this on the PC before launching the Simulation or Controller Node
# It starts the FastDDS Discovery Server bound to the Tailscale IP and exports the ROS_DISCOVERY_SERVER variable

PC_TAILSCALE_IP="100.122.20.128"

echo "Starting Fast DDS Discovery Server on ${PC_TAILSCALE_IP}:11811..."
# Run the discovery server in the background
fastdds discovery -i 0 -l ${PC_TAILSCALE_IP} -p 11811 &
DISCOVERY_PID=$!

echo "Fast DDS Server running (PID: $DISCOVERY_PID)"
echo "To connect to this server from this PC or the Raspberry Pi, export the following variable in your terminal:"
echo ""
echo "export ROS_DISCOVERY_SERVER=\"${PC_TAILSCALE_IP}:11811\""
echo ""
echo "Press Ctrl+C to stop the discovery server."

wait $DISCOVERY_PID
