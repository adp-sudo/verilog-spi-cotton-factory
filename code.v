`timescale 1ns/1ps

// ============================================================
// COTTON CUTTING MACHINE - SPI SLAVE
// SPI MODE 0: CPOL = 0, CPHA = 0
// Status: 01 = cutting, 02 = resting, 03 = requesting rest
// Command: D0 = poll status, B0 = grant rest
// ============================================================

module cotton_slave(
    input clk,
    input sclk,
    input cs_n,
    input mosi,
    input [3:0] batch_size,
    input [15:0] rest_duration,
    output reg miso,
    output reg cutting,
    output reg request_rest,
    output reg [31:0] total_cotton
);

    reg [3:0] cotton_count;
    reg [3:0] cut_timer;
    reg [15:0] rest_timer;

    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    reg [3:0] bit_count;

    reg grant_toggle;
    reg grant_seen;

    reg [7:0] status_byte;

    initial begin
        cutting = 1'b1;
        request_rest = 1'b0;

        cotton_count = 0;
        cut_timer = 0;
        rest_timer = 0;
        total_cotton = 0;

        tx_shift = 0;
        rx_shift = 0;
        bit_count = 0;

        grant_toggle = 0;
        grant_seen = 0;
        miso = 0;
        status_byte = 8'h01;
    end

    // Load the current status when this slave is selected.
    // The first status bit is available before the first SCLK edge.
    always @(negedge cs_n) begin
        bit_count = 0;
        rx_shift = 0;

        if (cutting == 1'b0)
            status_byte = 8'h02;
        else if (request_rest == 1'b1)
            status_byte = 8'h03;
        else
            status_byte = 8'h01;

        tx_shift = status_byte;
        miso = status_byte[7];
    end

    // Receive one command byte on rising SCLK edges.
    always @(posedge sclk) begin
        if (cs_n == 1'b0) begin
            rx_shift <= {rx_shift[6:0], mosi};

            if (bit_count == 7) begin
                if ({rx_shift[6:0], mosi} == 8'hB0)
                    grant_toggle <= ~grant_toggle;
            end

            bit_count <= bit_count + 1'b1;
        end
    end

    // Shift the status byte out on falling SCLK edges.
    always @(negedge sclk) begin
        if (cs_n == 1'b0) begin
            tx_shift <= {tx_shift[6:0], 1'b0};
            miso <= tx_shift[6];
        end
    end

    // Cotton cutting and fixed-duration resting operation.
    always @(posedge clk) begin

        if (grant_toggle != grant_seen) begin
            grant_seen <= grant_toggle;

            if (cutting == 1'b1 && request_rest == 1'b1) begin
                cutting <= 1'b0;
                request_rest <= 1'b0;
                rest_timer <= rest_duration;
            end
        end

        else if (cutting == 1'b0) begin

            if (rest_timer > 0)
                rest_timer <= rest_timer - 1'b1;

            if (rest_timer == 1) begin
                cutting <= 1'b1;
                cotton_count <= 0;
                cut_timer <= 0;
            end
        end

        else begin

            if (cut_timer == 4) begin
                cut_timer <= 0;
                total_cotton <= total_cotton + 1'b1;

                if (cotton_count == batch_size - 1'b1) begin
                    cotton_count <= 0;
                    request_rest <= 1'b1;
                end
                else begin
                    cotton_count <= cotton_count + 1'b1;
                end
            end

            else begin
                cut_timer <= cut_timer + 1'b1;
            end

        end
    end

endmodule


// ============================================================
// SPI MASTER AND SCHEDULER
// Polls all slaves, then grants rest while keeping at least
// one machine cutting.
// ============================================================

module spi_master(
    input clk,
    input miso,
    output reg sclk,
    output reg mosi,
    output reg [2:0] cs_n
);

    reg [3:0] state;
    reg [3:0] return_state;

    reg [1:0] slave_index;
    reg [1:0] decision_index;

    reg [1:0] active_count;

    reg [3:0] bit_count;
    reg phase;

    reg [7:0] tx_shift;
    reg [7:0] rx_shift;

    reg [7:0] status1;
    reg [7:0] status2;
    reg [7:0] status3;

    initial begin
        state = 0;
        return_state = 0;

        slave_index = 0;
        decision_index = 0;
        active_count = 0;

        sclk = 0;
        mosi = 0;
        cs_n = 3'b111;

        bit_count = 0;
        phase = 0;

        tx_shift = 0;
        rx_shift = 0;

        status1 = 8'h01;
        status2 = 8'h01;
        status3 = 8'h01;
    end

    always @(posedge clk) begin

        // STATE 0: Select a slave and start a status poll.
        if (state == 0) begin

            if (slave_index == 0)
                cs_n <= 3'b110;
            else if (slave_index == 1)
                cs_n <= 3'b101;
            else
                cs_n <= 3'b011;

            sclk <= 0;
            tx_shift <= 8'hD0;
            mosi <= 1'b1;

            rx_shift <= 0;
            bit_count <= 0;
            phase <= 0;

            return_state <= 2;
            state <= 1;
        end

        // STATE 1: Transfer one byte using SPI Mode 0.
        else if (state == 1) begin

            if (phase == 0) begin
                // Rising SCLK edge: slave samples MOSI.
                sclk <= 1;
                rx_shift <= {rx_shift[6:0], miso};
                phase <= 1;
            end

            else begin
                // Falling SCLK edge: prepare next MOSI bit.
                sclk <= 0;
                phase <= 0;

                if (bit_count == 7) begin
                    cs_n <= 3'b111;
                    mosi <= 0;
                    state <= return_state;
                end

                else begin
                    bit_count <= bit_count + 1'b1;
                    tx_shift <= {tx_shift[6:0], 1'b0};
                    mosi <= tx_shift[6];
                end
            end
        end

        // STATE 2: Save the received status byte.
        else if (state == 2) begin

            if (slave_index == 0)
                status1 <= rx_shift;
            else if (slave_index == 1)
                status2 <= rx_shift;
            else
                status3 <= rx_shift;

            if (slave_index == 2) begin
                state <= 3;
            end
            else begin
                slave_index <= slave_index + 1'b1;
                state <= 0;
            end
        end

        // STATE 3: Count the machines that are not resting.
        else if (state == 3) begin

            active_count = 0;

            if (status1 != 8'h02)
                active_count = active_count + 1'b1;

            if (status2 != 8'h02)
                active_count = active_count + 1'b1;

            if (status3 != 8'h02)
                active_count = active_count + 1'b1;

            decision_index <= 0;
            state <= 4;
        end

        // STATE 4: Decide whether each rest request is safe.
        else if (state == 4) begin

            if ((decision_index == 0 && status1 == 8'h03) ||
                (decision_index == 1 && status2 == 8'h03) ||
                (decision_index == 2 && status3 == 8'h03)) begin

                if (active_count > 1) begin

                    if (decision_index == 0)
                        cs_n <= 3'b110;
                    else if (decision_index == 1)
                        cs_n <= 3'b101;
                    else
                        cs_n <= 3'b011;

                    sclk <= 0;
                    tx_shift <= 8'hB0;
                    mosi <= 1'b1;

                    rx_shift <= 0;
                    bit_count <= 0;
                    phase <= 0;

                    return_state <= 5;
                    state <= 1;
                end

                else begin
                    if (decision_index == 2) begin
                        slave_index <= 0;
                        state <= 0;
                    end
                    else begin
                        decision_index <= decision_index + 1'b1;
                    end
                end
            end

            else begin
                if (decision_index == 2) begin
                    slave_index <= 0;
                    state <= 0;
                end
                else begin
                    decision_index <= decision_index + 1'b1;
                end
            end
        end

        // STATE 5: A rest grant was transmitted.
        else if (state == 5) begin

            active_count = active_count - 1'b1;

            if (decision_index == 2) begin
                slave_index <= 0;
                state <= 0;
            end
            else begin
                decision_index <= decision_index + 1'b1;
                state <= 4;
            end
        end

    end

endmodule


// ============================================================
// TOP MODULE: ONE MASTER CONNECTED TO THREE SPI SLAVES
// ============================================================

module cotton_factory(
    input clk,

    output cut1,
    output cut2,
    output cut3,

    output [31:0] cotton1,
    output [31:0] cotton2,
    output [31:0] cotton3,
    output [31:0] total_cotton,

    output sclk,
    output mosi,
    output miso,
    output [2:0] cs_n
);

    wire miso1;
    wire miso2;
    wire miso3;

    wire serial_clock;
    wire serial_mosi;
    wire serial_miso;

    wire [2:0] chip_select;

    assign sclk = serial_clock;
    assign mosi = serial_mosi;
    assign miso = serial_miso;
    assign cs_n = chip_select;

    // Only the selected slave drives the shared MISO line.
    assign serial_miso =
        (chip_select[0] == 0) ? miso1 :
        (chip_select[1] == 0) ? miso2 :
        (chip_select[2] == 0) ? miso3 : 1'b0;

    assign total_cotton = cotton1 + cotton2 + cotton3;

    // SPI Master
    spi_master MASTER(
        .clk(clk),
        .miso(serial_miso),
        .sclk(serial_clock),
        .mosi(serial_mosi),
        .cs_n(chip_select)
    );

    // Slave 1: 3 cotton units per batch, 20-cycle rest.
    cotton_slave SLAVE1(
        .clk(clk),
        .sclk(serial_clock),
        .cs_n(chip_select[0]),
        .mosi(serial_mosi),
        .batch_size(4'd3),
        .rest_duration(16'd20),
        .miso(miso1),
        .cutting(cut1),
        .request_rest(),
        .total_cotton(cotton1)
    );

    // Slave 2: 4 cotton units per batch, 20-cycle rest.
    cotton_slave SLAVE2(
        .clk(clk),
        .sclk(serial_clock),
        .cs_n(chip_select[1]),
        .mosi(serial_mosi),
        .batch_size(4'd4),
        .rest_duration(16'd20),
        .miso(miso2),
        .cutting(cut2),
        .request_rest(),
        .total_cotton(cotton2)
    );

    // Slave 3: 5 cotton units per batch, 20-cycle rest.
    cotton_slave SLAVE3(
        .clk(clk),
        .sclk(serial_clock),
        .cs_n(chip_select[2]),
        .mosi(serial_mosi),
        .batch_size(4'd5),
        .rest_duration(16'd20),
        .miso(miso3),
        .cutting(cut3),
        .request_rest(),
        .total_cotton(cotton3)
    );

endmodule
