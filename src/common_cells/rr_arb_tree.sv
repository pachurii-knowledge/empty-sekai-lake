`default_nettype none

module rr_arb_tree #(
    parameter int unsigned NumIn = 1,
    parameter type DataType = logic,
    parameter bit AxiVldRdy = 1'b0,
    parameter int unsigned IdxWidth = (NumIn <= 1) ? 1 : $clog2(NumIn)
) (
    input  logic                 clk_i,
    input  logic                 rst_ni,
    input  logic                 flush_i,
    input  logic [IdxWidth-1:0]  rr_i,
    input  logic [NumIn-1:0]     req_i,
    output logic [NumIn-1:0]     gnt_o,
    input  DataType [NumIn-1:0]  data_i,
    input  logic                 gnt_i,
    output logic                 req_o,
    output DataType              data_o,
    output logic [IdxWidth-1:0]  idx_o
);

generate
if (AxiVldRdy) begin : gen_axi_vld_rdy
    logic                     valid_q;
    DataType                  data_q;
    logic [IdxWidth-1:0]      idx_q;
    DataType                  selected_data;
    logic [IdxWidth-1:0]      selected_idx;
    logic [NumIn-1:0]         selected_gnt;
    logic                     selected_valid;

    always_comb begin
        logic found;
        selected_gnt = '0;
        selected_data = '0;
        selected_idx = '0;
        selected_valid = 1'b0;
        found = 1'b0;

        for (int unsigned offset = 0; offset < NumIn; offset += 1) begin
            int unsigned idx;
            idx = (int'(rr_i) + offset) % NumIn;
            if (req_i[idx] && !found) begin
                selected_idx = IdxWidth'(idx);
                selected_data = data_i[idx];
                selected_gnt[idx] = 1'b1;
                selected_valid = 1'b1;
                found = 1'b1;
            end
        end
    end

    assign gnt_o = (!valid_q) ? selected_gnt : '0;
    assign req_o = valid_q;
    assign data_o = data_q;
    assign idx_o = idx_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_q <= 1'b0;
            data_q <= '0;
            idx_q <= '0;
        end else if (flush_i) begin
            valid_q <= 1'b0;
            data_q <= '0;
            idx_q <= '0;
        end else begin
            if (valid_q && gnt_i) begin
                valid_q <= 1'b0;
            end
            if (!valid_q && selected_valid) begin
                valid_q <= 1'b1;
                data_q <= selected_data;
                idx_q <= selected_idx;
            end
        end
    end
end else begin : gen_comb
    always_comb begin
        logic found;
        gnt_o = '0;
        req_o = |req_i;
        data_o = '0;
        idx_o = '0;
        found = 1'b0;

        for (int unsigned offset = 0; offset < NumIn; offset += 1) begin
            int unsigned idx;
            idx = (int'(rr_i) + offset) % NumIn;
            if (req_i[idx] && !found) begin
                idx_o = IdxWidth'(idx);
                data_o = data_i[idx];
                gnt_o[idx] = gnt_i;
                found = 1'b1;
            end
        end
    end
end
endgenerate

endmodule: rr_arb_tree
