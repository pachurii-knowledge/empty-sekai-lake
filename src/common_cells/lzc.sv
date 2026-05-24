`default_nettype none

module lzc #(
    parameter int unsigned WIDTH = 1,
    parameter bit MODE = 1'b1,
    localparam int unsigned CNT_WIDTH = (WIDTH <= 1) ? 1 : $clog2(WIDTH + 1)
) (
    input  logic [WIDTH-1:0]     in_i,
    output logic [CNT_WIDTH-1:0] cnt_o,
    output logic                 empty_o
);

    always_comb begin
        cnt_o = '0;
        empty_o = 1'b1;
        if (MODE) begin
            for (int i = int'(WIDTH) - 1; i >= 0; i -= 1) begin
                if (in_i[i]) begin
                    cnt_o = CNT_WIDTH'(int'(WIDTH) - 1 - i);
                    empty_o = 1'b0;
                    break;
                end
            end
        end else begin
            for (int i = 0; i < int'(WIDTH); i += 1) begin
                if (in_i[i]) begin
                    cnt_o = CNT_WIDTH'(i);
                    empty_o = 1'b0;
                    break;
                end
            end
        end
        if (empty_o) begin
            cnt_o = CNT_WIDTH'(WIDTH);
        end
    end

endmodule: lzc
