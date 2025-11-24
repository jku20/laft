# Protocol
The laft usb application must communicate with the logic analyzer. They do it with the following protocol sent through USB bulk transfers. The laft application, a usb host, sends requests and the logic analyzer, a usb device, sends responses. There is no channel for the logic analyzer to make requests of its host. Each request is a 64 bit packet **starting with a 1 byte opcode**. The rest of request varies between opcodes. The format and size of a response depends on it's corresponding request.

## Opcodes
### Trace Request
#### Request
|0x01|0xSSSS|0xXXXXXXXXXX|

This request is used to dump data from every trace attached to the logic analyzer.

A trace request starts with a 1 byte opcode, 0x01. It then follows with a 2 byte big endian integer, the number of *bits* of data being requested *from each trace*. It then ends with 5 don't care bytes.

#### Response
The response will be a series of 8 byte responses, the raw data form each trace. Bits from each every trace are packaged into 2 bytes (traces indexed lower being first) and then this packed data is sequenced. It is sent over 64 bit responses. The final response may be incomplete, in which case the final bytes will be don't cares.
