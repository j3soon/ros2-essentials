#!/bin/bash -e

# Change the ownership of the directories that mounted from the host.
sudo chown -R ubuntu /home/ubuntu/.cache
sudo chown -R ubuntu /home/ubuntu/.nv
sudo chown -R ubuntu /home/ubuntu/.nvidia-omniverse
sudo chown -R ubuntu /home/ubuntu/.local
sudo chown -R ubuntu /home/ubuntu/Documents