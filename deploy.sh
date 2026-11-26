# !/bin/bash
set -euo pipefail

DEPLOY="./deploy"

if [-d "$DEPLOY"]; then
  mkdir ./deploy
fi

docker buildx build --platform linux/arm64 -t my_ros2_app:latest --output type=docker,dest=deploy/ros2_app.tar .

rsync -avz --exclude build --exclude install --exclude log \
  ~/ws/src/ jetson:~/ws/src/
ssh jetson@$s1 tar -xzf ./deploy/ros2_app.tar