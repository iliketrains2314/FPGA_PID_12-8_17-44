// Measures HIGH time of sig_in (posedge -> negedge)
// 'period' = number of clk cycles that sig_in stayed high
// 'valid'  = 1 for one clk when 'period' is updated
module bemf_speed_estimator (
    input  wire        clk,        // system clock
    input  wire        rst,        // sync reset, active high
    input  wire        sig_in,     // BEMF comparator output
    output reg  [31:0] period,     // measured high-time in clk cycles
    output reg         valid       // 1 clk pulse when 'period' is updated
);

    reg        sig_in_d;   // delayed input for edge detect
    wire       sig_in_rise;
    wire       sig_in_fall;
    reg [31:0] counter;    // counts while we're between rise and fall

    // Register previous value of sig_in
    always @(posedge clk) begin
        if (rst)
            sig_in_d <= 1'b0;
        else
            sig_in_d <= sig_in;
    end

    // Edge detection
    assign sig_in_rise =  sig_in & ~sig_in_d;  // 0 -> 1
    assign sig_in_fall = ~sig_in &  sig_in_d;  // 1 -> 0

    // Counter + period latch + valid pulse
    always @(posedge clk) begin
        if (rst) begin
            counter <= 32'd0;
            period  <= 32'd0;
            valid   <= 1'b0;
        end else if (sig_in_rise) begin
            // Start of high pulse: reset counter
            counter <= 32'd0;
            valid   <= 1'b0;
        end else begin
            // Free-run counter
            counter <= counter + 32'd1;

            if (sig_in_fall) begin
                // End of high pulse: latch high-time
                // '+1' so you count the cycle where the fall is detected
                period <= counter + 32'd1;
                valid  <= 1'b1;   // new measurement available
            end else begin
                valid  <= 1'b0;
            end
        end
    end

endmodule
