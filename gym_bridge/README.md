# Instructions

1. Replace params.yaml and scripts/gym_bridge.py with the files here
2. Put ros_agent.py in the scripts folder
3. Rebuild docker container
4. It should now run the agent publishing/subscribing to the following topics

## Subscribe topics
- Scan : "/opp_scan"
- Odom : "/opp_odom"

## Publish topic
- Drive : "/opp_drive"
