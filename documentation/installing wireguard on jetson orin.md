# Problem
The wireguard kernel module in not included by default on jetson AGX orin, so instead compiling it youself you can also use a non-kernel version of wireguard. This file contains instructions of how to do that.

## Step 1: Remove old versions of wireguard if they exist.


```bash
sudo apt-get purge -y wireguard-dkms
```

## Step 2: Install wireguard-go and wireguard-tools
```bash
sudo apt install wireguard-tools wireguard-go
```

## Step 3: Copy wireguard config file and enable wireguard service
```bash
sudo sudo cp <wireguard_file.conf /etc/wireguard/
sudo systemctl enable wg-quick@<wireguard_file>.service
sudo systemctl daemon-reload
```

## Step 4: Add env variable to the service to tell it to use the userspace version of wireguard
```bash
sudo vim /usr/lib/systemd/system/wg-quick@.service
```
Add:
```Bash
Environment=export WG_QUICK_USERSPACE_IMPLEMENTATION=wireguard
```

## Step 5: Reload / reboot
```bash
sudo systemctl start wg-quick@<wireguard_file>
```