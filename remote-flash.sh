# !/bin/bash

if [ -z "$DEPLOY_HOST" ]; then
  echo 'Please define DEPLOY_HOST by doing: export DEPLOY_HOST="device-ip";'
else 
  nixos-anywhere --flake .#nixos nixos@$DEPLOY_HOST
fi