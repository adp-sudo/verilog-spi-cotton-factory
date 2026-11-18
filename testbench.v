`timescale 1ns/1ps

module testbench;

    reg clk;

    wire cut1;
    wire cut2;
    wire cut3;

    wire [31:0] cotton1;
    wire [31:0] cotton2;
    wire [31:0] cotton3;
    wire [31:0] total_cotton;

    wire sclk;
    wire mosi;
    wire miso;
    wire [2:0] cs_n;

    wire [1:0] active_machines;

    reg safety_error;

    assign active_machines =
        {1'b0, cut1} +
        {1'b0, cut2} +
        {1'b0, cut3};

    cotton_factory DUT(
        .clk(clk),

        .cut1(cut1),
        .cut2(cut2),
        .cut3(cut3),

        .cotton1(cotton1),
        .cotton2(cotton2),
        .cotton3(cotton3),
        .total_cotton(total_cotton),

        .sclk(sclk),
        .mosi(mosi),
        .miso(miso),
        .cs_n(cs_n)
    );

    // 100 MHz simulation clock.
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        safety_error = 0;
    end

    // Check that at least one machine is cutting.
    always @(posedge clk) begin
        #1;

        if (active_machines == 0)
            safety_error = 1;
    end

    // Print production and machine status when they change.
    initial begin
        $monitor(
            "TIME=%0t | CUTTERS=%b%b%b | ACTIVE=%0d | S1=%0d S2=%0d S3=%0d | TOTAL=%0d | SAFETY_ERROR=%b",
            $time,
            cut3, cut2, cut1,
            active_machines,
            cotton1,
            cotton2,
            cotton3,
            total_cotton,
            safety_error
        );

        #5000;
        $finish;
    end

endmodule
