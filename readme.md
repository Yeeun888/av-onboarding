# What is this
AV onboarding task, demonstates a simple two node setup where 

**Node 1 (Main Node)**

Has two main goals: 
1. Broadcasting an encrypted message (through caesar cipher) to a topic with a key
2. Keeping track of the *unencrypted* message
3. Host a service where nodes can call upon it to check if their unencryption algorithm is correct

**Node 2 (Participant Node)**

1. Subscribe to the topic
2. Decrypt message
3. Call upon the service to verify itself

## Point of Improvement / Bugs
1. Current approach just stores 1 last known message. Multiple nodes would definitely kill the service
2. Launch file launches just one. Maybe having two would be good 

# How to setup
Clone the repository
```
git clone git@github.com:Yeeun888/av-onboarding.git
```
In the av-onboarding root run 
```
colcon build
```
To launch the main node do 
```
ros2 launch cipher_node cipher_launch.yaml.
Alternatively, you can launch the packages individually 
```
```
ros2 run cipher_node cipher_node
ros2 run cipher_node participant_node
