// udp_ipv4.vh - shared Ethernet II / IPv4 / UDP header logic for the packetiser (inferno_tx.v).
//
// Provenance: written for OpenProSeries (W16) from IEEE 802.3 (Ethernet II), RFC 791 (IPv4 header: version 4,
// IHL 5, DF, no options), RFC 1071 (the header checksum) and RFC 768 (UDP; checksum 0 = none over IPv4).
// No vendor material. Licence: GPL-3.0.
//
// Include inside a module. Both functions are pure combinational logic.

// IPv4 header checksum of a 20-byte header 45 tos | len | id | 4000 | ttl 11 | csum | src | dst
function [15:0] ipv4_csum;
    input [7:0]  h_tos;
    input [15:0] h_len;
    input [15:0] h_id;
    input [7:0]  h_ttl;
    input [31:0] h_src;
    input [31:0] h_dst;
    reg   [19:0] s;
    reg   [16:0] f;
    begin
        s = {4'd0, 8'h45, h_tos} + h_len + h_id + 16'h4000 + {h_ttl, 8'd17}
          + h_src[31:16] + h_src[15:0] + h_dst[31:16] + h_dst[15:0];
        f = s[15:0] + s[19:16];
        ipv4_csum = ~(f[15:0] + f[16]);
    end
endfunction

// byte i (0..41) of the Ethernet II + IPv4 + UDP header
function [7:0] eth_ipv4_udp_byte;
    input [5:0]  h_i;
    input [47:0] h_dmac;
    input [47:0] h_smac;
    input [7:0]  h_tos;
    input [15:0] h_iplen;
    input [15:0] h_id;
    input [7:0]  h_ttl;
    input [15:0] h_csum;
    input [31:0] h_sip;
    input [31:0] h_dip;
    input [15:0] h_sport;
    input [15:0] h_dport;
    input [15:0] h_ulen;
    begin
        case (h_i)
            6'd0:  eth_ipv4_udp_byte = h_dmac[47:40]; 6'd1:  eth_ipv4_udp_byte = h_dmac[39:32];
            6'd2:  eth_ipv4_udp_byte = h_dmac[31:24]; 6'd3:  eth_ipv4_udp_byte = h_dmac[23:16];
            6'd4:  eth_ipv4_udp_byte = h_dmac[15:8];  6'd5:  eth_ipv4_udp_byte = h_dmac[7:0];
            6'd6:  eth_ipv4_udp_byte = h_smac[47:40]; 6'd7:  eth_ipv4_udp_byte = h_smac[39:32];
            6'd8:  eth_ipv4_udp_byte = h_smac[31:24]; 6'd9:  eth_ipv4_udp_byte = h_smac[23:16];
            6'd10: eth_ipv4_udp_byte = h_smac[15:8];  6'd11: eth_ipv4_udp_byte = h_smac[7:0];
            6'd12: eth_ipv4_udp_byte = 8'h08;       6'd13: eth_ipv4_udp_byte = 8'h00;       // IPv4
            6'd14: eth_ipv4_udp_byte = 8'h45;       6'd15: eth_ipv4_udp_byte = h_tos;
            6'd16: eth_ipv4_udp_byte = h_iplen[15:8]; 6'd17: eth_ipv4_udp_byte = h_iplen[7:0];
            6'd18: eth_ipv4_udp_byte = h_id[15:8];    6'd19: eth_ipv4_udp_byte = h_id[7:0];
            6'd20: eth_ipv4_udp_byte = 8'h40;       6'd21: eth_ipv4_udp_byte = 8'h00;       // DF, offset 0
            6'd22: eth_ipv4_udp_byte = h_ttl;         6'd23: eth_ipv4_udp_byte = 8'd17;       // UDP
            6'd24: eth_ipv4_udp_byte = h_csum[15:8];  6'd25: eth_ipv4_udp_byte = h_csum[7:0];
            6'd26: eth_ipv4_udp_byte = h_sip[31:24];  6'd27: eth_ipv4_udp_byte = h_sip[23:16];
            6'd28: eth_ipv4_udp_byte = h_sip[15:8];   6'd29: eth_ipv4_udp_byte = h_sip[7:0];
            6'd30: eth_ipv4_udp_byte = h_dip[31:24];  6'd31: eth_ipv4_udp_byte = h_dip[23:16];
            6'd32: eth_ipv4_udp_byte = h_dip[15:8];   6'd33: eth_ipv4_udp_byte = h_dip[7:0];
            6'd34: eth_ipv4_udp_byte = h_sport[15:8]; 6'd35: eth_ipv4_udp_byte = h_sport[7:0];
            6'd36: eth_ipv4_udp_byte = h_dport[15:8]; 6'd37: eth_ipv4_udp_byte = h_dport[7:0];
            6'd38: eth_ipv4_udp_byte = h_ulen[15:8];  6'd39: eth_ipv4_udp_byte = h_ulen[7:0];
            default: eth_ipv4_udp_byte = 8'h00;                                              // 40, 41: no UDP checksum
        endcase
    end
endfunction
