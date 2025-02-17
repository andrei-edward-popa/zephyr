# mcumgr

This is mcumgr, version 0.2.0

mcumgr is a management library for 32-bit MCUs.   The goal of mcumgr is to
define a common management infrastructure with pluggable transport and encoding
components.  In addition, mcumgr provides definitions and handlers for some
core commands: image management, file system management, and OS management.

mcumgr is operating system and hardware independent.  It relies on hardware
porting layers from the operating system it runs on.

## Configuration

The `samples/smp_svr/zephyr/prj.conf` file provides a good starting point for
configuring an application to use *mcumgr*.  The major configuration settings
are described below:

| Setting       | Description   | Default |
| ------------- | ------------- | ------- |
| `CONFIG_MCUMGR` | Enable the mcumgr management library. | n |
| `CONFIG_MCUMGR_CMD_FS_MGMT` | Enable mcumgr handlers for file management | n |
| `CONFIG_MCUMGR_CMD_IMG_MGMT` | Enable mcumgr handlers for image management | n |
| `CONFIG_MCUMGR_CMD_OS_MGMT` | Enable mcumgr handlers for OS management | n |
| `CONFIG_MCUMGR_CMD_STAT_MGMT` | Enable mcumgr handlers for statistics | n |
| `CONFIG_MCUMGR_GRP_ZEPHYR_BASIC` | Enable mcumgr basic commands group | n |

## Dependencies

To use mcumgr's image management support, your device must be running version
1.1.0 or later of the [MCUboot boot
loader](https://github.com/mcu-tools/mcuboot).  The other mcumgr features do
not require MCUboot.

## Command line tool

The `mcumgr` command line tool is available at:
https://github.com/apache/mynewt-mcumgr-cli.  The command line tool requires [Go
1.12 or later](https://golang.org/dl/).  Once Go is installed and set up on your
system, you can install the mcumgr CLI tool by issuing the following `go get`
command:

```
$ go get github.com/apache/mynewt-mcumgr-cli/mcumgr
```

The `mcumgr` tool allows you to manage devices running an mcumgr server.

## Architecture

The mcumgr stack has the following layout:

```
+---------------------+---------------------+
|             <command handlers>            |
+---------------------+---------------------+
|                   mgmt                    |
+---------------------+---------------------+
|           <transfer encoding(s)>          |
+---------------------+---------------------+
|               <transport(s)>              |
+---------------------+---------------------+
```

Items enclosed in angled brackets represent generic components that can be plugged into mcumgr.  The items in this stack diagram are defined below:
* *Command handler*: Processes incoming mcumgr requests and generates corresponding responses.  A command handler is associated with a single command type, defined by a (group ID, command ID) pair.
* *mgmt*: The core of mcumgr; facilitates the passing of requests and responses between the generic command handlers and the concrete transports and transfer encodings.
* *Transfer encoding*: Defines how mcumgr requests and responses are encoded on the wire.
* *Transport*: Sends and receives mcumgr packets over a particular medium.

Each transport is configured with a single transfer encoding.

As an example, the sample application `smp_svr` uses the following components:

* Command handlers:
    * Image management (`img_mgmt`)
    * File system management (`fs_mgmt`)
    * OS management (`os_mgmt`)
* Transfer/Transports protocols:
    * SMP/Bluetooth
    * SMP/Shell

yielding the following stack diagram:

```
+----------+----------+----------+----------+
| img_mgmt |  fs_mgmt |  os_mgmt |   ...    |
+----------+----------+----------+----------+
|                   mgmt                    |
+---------------------+---------------------+
|         SMP         |         SMP         |
+---------------------+---------------------+
|      Bluetooth      |        Shell        |
+---------------------+---------------------+
```

## Command definition

An mcumgr request or response consists of the following two components:
* mcumgr header
* CBOR key-value map

How these two components are encoded and parsed depends on the transfer
encoding used.

The mcumgr header structure is defined in `subsys/mgmt/mcumgr/smp_internal.h`
as `struct smp_hdr`.

The contents of the CBOR key-value map are specified per command type.

## Supported transfer encodings

Mcumgr comes with one built-in transfer encoding: Simple Management Protocol
(SMP).  SMP requests and responses have a very basic structure.  For details,
see the comments at the top of `smp/include/smp/smp.h`.

## Supported transports

The mcumgr project defines two transports:
* [SMP/Console](transport/smp-console.md)
* [SMP/Bluetooth](transport/smp-bluetooth.md)

Implementations, being hardware- and OS-specific, are not included.

## Browsing

Information and documentation for mcumgr is stored within the source.

For more information in the source, here are some pointers:

- [cborattr](cborattr): Used for parsing incoming mcumgr requests.  Destructures mcumgr packets and populates corresponding field variables.
- [cmd](cmd): Built-in command handlers for the core mcumgr commands.
- [ext](ext): Third-party libraries that mcumgr depends on.
- [mgmt](mgmt): Code implementing the `mgmt` layer of mcumgr.
- [smp](smp): The built-in transfer encoding: Simple management protocol.

## Joining

Developers welcome!

* Discord mcumgr channel: https://discord.com/invite/Ck7jw53nU2
