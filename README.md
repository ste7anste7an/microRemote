# UARTRemote for MicroBlocks = MicroUART

This is a stripped down version of our UARTRemote library.

## Types
The following types are supported:
- Number
- String
- Boolean
- Bytearay

## Protocol

```
<tot_len> <PREAMBLE> <len_cmd> <cmd> [<data_type> <data_len> <data>]
with
- tot_len: total length of frame
- PREAMBLE: uniq preamble: '<$MU'
- len_cmd: length of command string
- cmd: command string
- [ ] any number
- data_type: one of 'N' (number), 'S' (string), 'A' (bytearray), 'B' (boolean)
- data_len: length of data
- data: encoded data. Numbers as strings encoded as UTF-8 , Struing encoded as UTF-8, boolean as '\x00' and '\x01' for False resp True.

## robust receive
The receive function collects bytes from the UART 1 by 1 until the time_out is exceeded. Per byte it has a timeout of byte_timeout which is fixed at 10ms.

When receiving the first 4 bytes of the frame, the PERAMBLE is checked. If there is a mistake,the UART buffer is cleared (read_all()).
