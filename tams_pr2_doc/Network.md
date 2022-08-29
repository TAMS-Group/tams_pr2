## Overall network setup

[Network Setup](https://gogs.crossmodal-learning.org/TAMS/tams_pr2/wiki/Network-Setup/)


## Accessing the PR2 from external machines other than the basestation

### Via the network gateway (preferred for machines with 134.100.13.XXX IPs)

#### Add the new machine to the VPN whitelist

* Log into basestation
* For temporary access, execute
    * `sudo iptables -t filter -I FORWARD -d 134.100.13.XXX/32 -j ACCEPT`
    * `sudo iptables -t filter -I FORWARD -s 134.100.13.XXX/32 -j ACCEPT`
* For permanent access, edit `/etc/iptables/rules.v4`
    * find two lines looking like
    * `-A FORWARD -d 134.100.13.XXX/32 -j ACCEPT`
    * `-A FORWARD -s 134.100.13.XXX/32 -j ACCEPT`
    * above or below those lines, add the same two lines for your machine

#### Add the route to reach the PR2's internal network to the new machine

* These instructions have to be performed *on the new machine*, neither on the PR2, nor on the basestation
* For temporary routing, execute
    * `sudo ip route add 10.68.0.0/24 via 134.100.13.224`
* For permanent access, edit `/etc/network/interfaces`
    * add the following line and replace <ethernetInterface> to the real ethernetInterface name of *the new machine* (for example, eth0)
    * `ip route add -net 10.68.0.0/24 gw 134.100.13.224 dev <ethernetInterface>`

### Connect to the PR2 computers

```
$ export ROS_MASTER_URI=http://10.68.0.1:11311
$ rostopic echo /joint_states
```

Due to ROS-internal protocols, relying on DNS resolution, it should be **strongly** preferred to refer to the computers by name instead of IP and there might be some (undetected) issues when using only the IPs for ROS connections.
Please add the following lines to your computers `/etc/hosts` file:

```
10.68.0.1 c1 docker-c1
10.68.0.2 c2 docker-c2
10.68.0.10  pr2-head docker-pr2-head
```

with this local information, the following should work:

```
$ export ROS_MASTER_URI=http://c1:11311
$ rostopic echo /joint_states
```


## Through adding your computer to PR2's VPN

If a machine outside TAMS's laboratory routing subnet (basically the TAMS floor) should connect to the robot, it cannot use the above gateway-based method because it cannot directly connect to the base station and the university infrastructure does not forward robot packages to this custom gateway.
Instead, the machine has to be added as a VPN host at the cost of more network configuration.

Based on https://www.digitalocean.com/community/tutorials/how-to-set-up-an-openvpn-server-on-ubuntu-14-04#step-3-%E2%80%94-generate-certificates-and-keys-for-clients

Hint: the concat-ovpn.sh and the machine.ovpn can be found in CML gogs 'tams_pr2_scripts'. machine.ovpn is the client.conf with the ca, cert and key tag commented out

#### Create Client Key (on tams43)

* Log into basestation as root
* `cd /etc/openvpn/easy-rsa/`
* `source ./vars`
* `./build-key <machine_name>`
* Hit ENTER on all questions and for the last (certify) type 'y' and ENTER
* Go to the keys directory `cd keys`
* Build the client config with included keys `./concat-ovpn.sh <machine>`
    * If the client config and key already exists, simply copy it.

#### Copy Client Key to \<machine\>

* Log into the \<machine\>
* `scp root@tams43:/etc/openvpn/easy-rsa/keys/<machine>.ovpn Downloads/`

#### Enable ssh connection on VPN IP (on client)
* Edit `/etc/ssh/sshd_config`
* Comment out ListenAddress \<machine_ip\>
* Restart ssh with `service ssh restart`

#### Set ROS IP and connect (on client)
* Install openvpn if not installed yet
* Ensure that the ROS_IP on the target computer (the ROS master) is set to its VPN address (10.68.0.1 for the PR2)
* Connect to the VPN `sudo openvpn --config ~/Downloads/<machine>.ovpn`
* Find clients VPN IP from connection log or `ifconfig`. Alternatively set a fixed VPN IP for the client.
* Set the ROS_IP for the client to its VPN address export `ROS_IP=10.68.1.X`
* Set ROS master export `ROS_MASTER_URI=http://10.68.0.1:11311`
* Repeat the above 2 steps for all terminal windows

#### Set a fixed VPN IP for a client
* Create a file named \<machine\> inside `/etc/openvpn/ccd/`
* Add line `ifconfig-push 10.68.1.X 10.68.1.X+1`
* Fixed IP addresses must be in range 10.68.1.10 and 10.68.1.99
* Dynamic addresses will be 10.68.1.100 and above
* Ensure to have no conflicts with other fixed addresses

#### Revoke Access
* Log into basestation as root
* `cd /etc/openvpn/ccd`
* `cp disable <machine>`
