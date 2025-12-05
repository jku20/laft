# Protocol
The laft usb application must communicate with the logic analyzer. They do it with the following protocol sent through USB bulk transfers. The laft application, a usb host, sends requests and the logic analyzer, a usb device, sends responses. There is no channel for the logic analyzer to make requests of its host. Each request is a 64 bit packet **starting with a 1 byte opcode**. The rest of request varies between opcodes. The format and size of a response depends on it's corresponding request.

## Opcodes
### Trace Request
#### Request
|01|SSSS|XXXXXXXXXX|

This request is used to dump data from every trace attached to the logic analyzer.

A trace request starts with a 1 byte opcode, 0x01. It then follows with a 2 byte big endian integer, the number of *bits* of data being requested *from each trace*. It then ends with 5 don't care bytes.

#### Response
The response will be a series of 8 byte responses, the raw data form each trace. Bits from each every trace are packaged into 2 bytes (traces indexed lower being first) and then this packed data is sequenced. It is sent over 64 bit responses. The final response may be incomplete, in which case the final bytes will be don't cares.

### Frequency Set Request
#### Request
|02|XXXXXX|FFFFFFFF|

This request sets the frequency the logic analyzer will expect from its traces.

A frequency set request starts with a 1 byte opcode, 0x01. It then followed with 3 bytes of don't cares and ends with a 4 byte little endian integer, the frequency to listen at.

#### Response
The MCU is not expected to respond to this request.

### Rising Edge Trigger Request
#### Request
|04|X|P|SSSS|XXXXXXXX|

This request request is used to dump data from every trace attached to the logic analyzer starting on a rising edge for a pin specified by P. This is similar to a trace request, but instead of starting recording at some arbitrary point in time, it starts on a rising edge.

The format is composed of a one byte opcode, followed by a nibble of don't cares and then a 4 bit number, the pin of the logic analyzer to look for an edge on. This is followed by a little endian two byte size, the number of bits to record *from each trace*. The request is then padded by four bytes of don't cares.

#### Response
The response is a series of 8 byte responses, the raw data from each trace. It's format is the same as a Trace Request. Refer to that for more details.
