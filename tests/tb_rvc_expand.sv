/**
 * tb_rvc_expand.sv
 *
 * Standalone exhaustive dumper for the RV64C 16->32 expander. Drives all 2^16
 * parcels through rvc_expand and writes "cccc xxxxxxxx i" (hex parcel, hex
 * expanded word, illegal bit) per line to rvc_expand_dump.txt. The Python
 * harness (scripts/test_rvc_expand.py) diffs it against two independent goldens
 * (a Python reference expander + binutils objdump).
 */
`default_nettype none

module top;
    logic [15:0] c;
    logic [31:0] x;
    logic        illegal;

    rvc_expand dut (.c(c), .x(x), .illegal(illegal));

    integer fd;
    initial begin
        fd = $fopen("rvc_expand_dump.txt", "w");
        if (fd == 0) begin
            $display("FATAL: cannot open rvc_expand_dump.txt");
            $finish;
        end
        for (int i = 0; i < 65536; i += 1) begin
            c = i[15:0];
            #1;
            $fwrite(fd, "%04x %08x %0d\n", c, x, illegal);
        end
        $fclose(fd);
        $display("RVC_EXPAND_DUMP_DONE 65536");
        $finish;
    end
endmodule

`default_nettype wire
