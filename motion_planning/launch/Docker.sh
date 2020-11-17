#!/usr/bin/env bash
cd ~/crossbar-examples/getting-started/
sudo docker run -v  $PWD:/node -u 0 --rm --name=crossbar -it -p 8080:8080 crossbario/crossbar
