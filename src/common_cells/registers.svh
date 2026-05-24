`ifndef COMMON_CELLS_REGISTERS_SVH_
`define COMMON_CELLS_REGISTERS_SVH_

`define FF(q, d, reset_value) \
    always_ff @(posedge clk_i or negedge rst_ni) begin \
        if (!rst_ni) begin \
            q <= reset_value; \
        end else begin \
            q <= d; \
        end \
    end

`define FFL(q, d, load, reset_value) \
    always_ff @(posedge clk_i or negedge rst_ni) begin \
        if (!rst_ni) begin \
            q <= reset_value; \
        end else if (load) begin \
            q <= d; \
        end \
    end

`define FFLNR(q, d, load, clk) \
    always_ff @(posedge clk) begin \
        if (load) begin \
            q <= d; \
        end \
    end

`define FFLARNC(q, d, load, clear, reset_value, clk, rst_n) \
    always_ff @(posedge clk or negedge rst_n) begin \
        if (!rst_n) begin \
            q <= reset_value; \
        end else if (clear) begin \
            q <= reset_value; \
        end else if (load) begin \
            q <= d; \
        end \
    end

`endif
