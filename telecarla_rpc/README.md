# TELECARLA RPC

This package sends control commands from the operator client to the vehicle server using the [rpclib](https://github.com/rpclib/rpclib).

## Usage

Start the rpc server on the vehicle/simulator side with

```shell
roslaunch telecarla_rpc telecarla_rpc_server.launch
```

Connect the client from the operator side with

```shell
roslaunch telecarla_rpc telecarla_rpc_client.launch rpc_host:=IP
```
