#!/usr/bin/env bash

echo 'Install dependencies'

git clone https://github.com/rpclib/rpclib.git
mkdir -p rpclib/build
pushd rpclib/build || exit
cmake ..
cmake --build .
sudo make install
popd || exit
rm -rf rpclib
