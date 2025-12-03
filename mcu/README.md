# MCU Source
This is source code to flash the MCU.

## Design
We double buffer the streaming data in two circular buffers. When receiving a request and responding to it, we swap the buffer we are streaming to so as not to be reading and writing from the same buffer.
