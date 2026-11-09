// eth_crc32.vh - IEEE 802.3 CRC-32 (FCS), one byte at a time.
//
// Provenance: written for OpenProSeries from IEEE 802.3 clause 3.2.9 (polynomial 0x04C11DB7, bit-reversed
// 0xEDB88320, register preset to all ones, the FCS is the one's complement sent least-significant bit first).
// A receiver that runs the same register over data + FCS ends on the residue 0xDEBB20E3. No vendor material.
// Licence: GPL-3.0 (OpenProSeries).

function [31:0] crc32_byte;
    input [31:0] c;
    input [7:0]  d;
    integer      i;
    reg   [31:0] x;
    begin
        x = c ^ {24'd0, d};
        for (i = 0; i < 8; i = i + 1)
            x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
        crc32_byte = x;
    end
endfunction
