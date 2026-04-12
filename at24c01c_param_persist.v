// -----------------------------------------------------------------------------
// at24c01c_param_persist
// -----------------------------------------------------------------------------
// Purpose:
//   Automatic parameter persistence using AT24C01C over I2C.
//   1) Read once at power-up.
//   2) After each save_req pulse, write latest parameter snapshot.
//
// Notes:
//   - Device address: 0x50 (A2/A1/A0 grounded).
//   - Byte write mode is used (simple and robust).
//   - Write cycle wait time follows typical tWR ~5ms.
// -----------------------------------------------------------------------------
module at24c01c_param_persist #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer I2C_HZ = 100_000
)(
    input  wire        clk_50m,
    input  wire        rst_n,

    input  wire        save_req,
    input  wire [51:0] save_state,

    output reg         load_valid,
    output reg  [51:0] load_state,
    output reg         busy,
    output reg         error,

    output wire        scl,
    input  wire        sda_in,
    output wire        sda_out,
    output wire        sda_oe
);

    localparam [7:0] DEV_ADDR_W = 8'hA0;
    localparam [7:0] DEV_ADDR_R = 8'hA1;
    localparam [7:0] MAGIC      = 8'hA5;
    localparam [7:0] VERSION    = 8'h01;

    localparam integer TICK_DIV = CLK_HZ / (I2C_HZ * 2); // half SCL period
    localparam integer TWR_WAIT_CYC = CLK_HZ / 200;      // ~5ms

    // Primitive op types.
    localparam [2:0] PRIM_NONE  = 3'd0;
    localparam [2:0] PRIM_START = 3'd1;
    localparam [2:0] PRIM_STOP  = 3'd2;
    localparam [2:0] PRIM_WRITE = 3'd3;
    localparam [2:0] PRIM_READ  = 3'd4;

    // High-level states.
    localparam [5:0] HS_BOOT_SET_START = 6'd0;
    localparam [5:0] HS_BOOT_SET_DEVW  = 6'd1;
    localparam [5:0] HS_BOOT_SET_ADDR0 = 6'd2;
    localparam [5:0] HS_BOOT_SET_STOP  = 6'd3;
    localparam [5:0] HS_BOOT_RD_START  = 6'd4;
    localparam [5:0] HS_BOOT_RD_DEVR   = 6'd5;
    localparam [5:0] HS_BOOT_RD_BYTE   = 6'd6;
    localparam [5:0] HS_BOOT_RD_STOP   = 6'd7;
    localparam [5:0] HS_BOOT_CHECK     = 6'd8;
    localparam [5:0] HS_IDLE           = 6'd9;
    localparam [5:0] HS_SAVE_PREP      = 6'd10;
    localparam [5:0] HS_SAVE_START     = 6'd11;
    localparam [5:0] HS_SAVE_DEVW      = 6'd12;
    localparam [5:0] HS_SAVE_ADDR      = 6'd13;
    localparam [5:0] HS_SAVE_DATA      = 6'd14;
    localparam [5:0] HS_SAVE_STOP      = 6'd15;
    localparam [5:0] HS_SAVE_WAIT      = 6'd16;

    // I2C line controls.
    reg scl_out;
    reg sda_drive_low;
    assign scl = scl_out;
    // Open-drain style:
    //   - sda_out is fixed to 0
    //   - sda_oe=1 drives low, sda_oe=0 releases line
    assign sda_out = 1'b0;
    assign sda_oe  = sda_drive_low;

    // Tick generator.
    reg [15:0] tick_cnt;
    wire bit_tick = (tick_cnt == (TICK_DIV - 1));

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            tick_cnt <= 16'd0;
        else if (bit_tick)
            tick_cnt <= 16'd0;
        else
            tick_cnt <= tick_cnt + 16'd1;
    end

    // Primitive engine.
    reg        prim_busy;
    reg        prim_done;
    reg [2:0]  prim_op;
    reg [2:0]  prim_state;
    reg        prim_phase;
    reg [3:0]  prim_bit;
    reg [7:0]  prim_shift;
    reg [7:0]  prim_rx_byte;
    reg        prim_ack_sample;
    reg        prim_ack_to_slave; // for PRIM_READ: 0=ACK, 1=NACK

    reg        prim_launch;
    reg [2:0]  prim_launch_op;
    reg [7:0]  prim_launch_byte;
    reg        prim_launch_ack;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            scl_out         <= 1'b1;
            sda_drive_low   <= 1'b0;
            prim_busy       <= 1'b0;
            prim_done       <= 1'b0;
            prim_op         <= PRIM_NONE;
            prim_state      <= 3'd0;
            prim_phase      <= 1'b0;
            prim_bit        <= 4'd0;
            prim_shift      <= 8'd0;
            prim_rx_byte    <= 8'd0;
            prim_ack_sample <= 1'b1;
            prim_ack_to_slave <= 1'b1;
        end else begin
            prim_done <= 1'b0;

            if (!prim_busy && prim_launch) begin
                prim_busy <= 1'b1;
                prim_op   <= prim_launch_op;
                prim_state <= 3'd0;
                prim_phase <= 1'b0;
                prim_bit   <= 4'd7;
                prim_shift <= prim_launch_byte;
                prim_ack_to_slave <= prim_launch_ack;
                prim_rx_byte <= 8'd0;
                prim_ack_sample <= 1'b1;
            end else if (prim_busy && bit_tick) begin
                case (prim_op)
                    PRIM_START: begin
                        case (prim_state)
                            3'd0: begin
                                scl_out       <= 1'b1;
                                sda_drive_low <= 1'b0;
                                prim_state    <= 3'd1;
                            end
                            3'd1: begin
                                scl_out       <= 1'b1;
                                sda_drive_low <= 1'b1;
                                prim_state    <= 3'd2;
                            end
                            default: begin
                                scl_out       <= 1'b0;
                                sda_drive_low <= 1'b1;
                                prim_busy     <= 1'b0;
                                prim_done     <= 1'b1;
                            end
                        endcase
                    end

                    PRIM_STOP: begin
                        case (prim_state)
                            3'd0: begin
                                scl_out       <= 1'b0;
                                sda_drive_low <= 1'b1;
                                prim_state    <= 3'd1;
                            end
                            3'd1: begin
                                scl_out       <= 1'b1;
                                sda_drive_low <= 1'b1;
                                prim_state    <= 3'd2;
                            end
                            default: begin
                                scl_out       <= 1'b1;
                                sda_drive_low <= 1'b0;
                                prim_busy     <= 1'b0;
                                prim_done     <= 1'b1;
                            end
                        endcase
                    end

                    PRIM_WRITE: begin
                        case (prim_state)
                            3'd0: begin
                                // Data bit loop.
                                if (!prim_phase) begin
                                    scl_out       <= 1'b0;
                                    sda_drive_low <= ~prim_shift[prim_bit];
                                    prim_phase    <= 1'b1;
                                end else begin
                                    scl_out    <= 1'b1;
                                    prim_phase <= 1'b0;
                                    if (prim_bit == 0)
                                        prim_state <= 3'd1;
                                    else
                                        prim_bit <= prim_bit - 1'b1;
                                end
                            end
                            3'd1: begin
                                // ACK sample.
                                if (!prim_phase) begin
                                    scl_out       <= 1'b0;
                                    sda_drive_low <= 1'b0;
                                    prim_phase    <= 1'b1;
                                end else begin
                                    scl_out         <= 1'b1;
                                    prim_ack_sample <= sda_in; // 0=ACK
                                    prim_phase      <= 1'b0;
                                    prim_state      <= 3'd2;
                                end
                            end
                            default: begin
                                scl_out       <= 1'b0;
                                sda_drive_low <= 1'b0;
                                prim_busy     <= 1'b0;
                                prim_done     <= 1'b1;
                            end
                        endcase
                    end

                    PRIM_READ: begin
                        case (prim_state)
                            3'd0: begin
                                // Data bit loop.
                                if (!prim_phase) begin
                                    scl_out       <= 1'b0;
                                    sda_drive_low <= 1'b0;
                                    prim_phase    <= 1'b1;
                                end else begin
                                    scl_out <= 1'b1;
                                    prim_rx_byte[prim_bit] <= sda_in;
                                    prim_phase <= 1'b0;
                                    if (prim_bit == 0)
                                        prim_state <= 3'd1;
                                    else
                                        prim_bit <= prim_bit - 1'b1;
                                end
                            end
                            3'd1: begin
                                // Master ACK/NACK.
                                if (!prim_phase) begin
                                    scl_out       <= 1'b0;
                                    sda_drive_low <= ~prim_ack_to_slave; // ACK=0 => drive low
                                    prim_phase    <= 1'b1;
                                end else begin
                                    scl_out    <= 1'b1;
                                    prim_phase <= 1'b0;
                                    prim_state <= 3'd2;
                                end
                            end
                            default: begin
                                scl_out       <= 1'b0;
                                sda_drive_low <= 1'b0;
                                prim_busy     <= 1'b0;
                                prim_done     <= 1'b1;
                            end
                        endcase
                    end

                    default: begin
                        prim_busy <= 1'b0;
                        prim_done <= 1'b1;
                    end
                endcase
            end
        end
    end

    // High-level controller.
    reg [5:0] hs_state;
    reg       hs_step_issued;
    reg [3:0] byte_idx;
    reg [21:0] wr_wait_cnt;
    reg        pending_save;
    reg [51:0] latched_save_state;
    reg        boot_done;

    reg [7:0] rd_buf [0:9];
    reg [7:0] wr_buf [0:9];
    reg [7:0] checksum;
    integer i;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            hs_state         <= HS_BOOT_SET_START;
            hs_step_issued   <= 1'b0;
            byte_idx         <= 4'd0;
            wr_wait_cnt      <= 22'd0;
            pending_save     <= 1'b0;
            latched_save_state <= 52'd0;
            boot_done        <= 1'b0;
            load_valid       <= 1'b0;
            load_state       <= 52'd0;
            busy             <= 1'b1;
            error            <= 1'b0;
            prim_launch      <= 1'b0;
            prim_launch_op   <= PRIM_NONE;
            prim_launch_byte <= 8'd0;
            prim_launch_ack  <= 1'b1;
            checksum         <= 8'd0;
            for (i = 0; i < 10; i = i + 1) begin
                rd_buf[i] <= 8'd0;
                wr_buf[i] <= 8'd0;
            end
        end else begin
            load_valid  <= 1'b0;
            prim_launch <= 1'b0;
            busy <= (hs_state != HS_IDLE);

            case (hs_state)
                // -------------------------
                // Boot read sequence.
                // -------------------------
                HS_BOOT_SET_START: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch    <= 1'b1;
                        prim_launch_op <= PRIM_START;
                        hs_step_issued <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        hs_state <= HS_BOOT_SET_DEVW;
                    end
                end

                HS_BOOT_SET_DEVW: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch      <= 1'b1;
                        prim_launch_op   <= PRIM_WRITE;
                        prim_launch_byte <= DEV_ADDR_W;
                        hs_step_issued   <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        if (prim_ack_sample)
                            hs_state <= HS_IDLE;
                        else
                            hs_state <= HS_BOOT_SET_ADDR0;
                    end
                end

                HS_BOOT_SET_ADDR0: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch      <= 1'b1;
                        prim_launch_op   <= PRIM_WRITE;
                        prim_launch_byte <= 8'h00;
                        hs_step_issued   <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        if (prim_ack_sample)
                            hs_state <= HS_IDLE;
                        else
                            hs_state <= HS_BOOT_SET_STOP;
                    end
                end

                HS_BOOT_SET_STOP: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch    <= 1'b1;
                        prim_launch_op <= PRIM_STOP;
                        hs_step_issued <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        hs_state <= HS_BOOT_RD_START;
                    end
                end

                HS_BOOT_RD_START: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch    <= 1'b1;
                        prim_launch_op <= PRIM_START;
                        hs_step_issued <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        hs_state <= HS_BOOT_RD_DEVR;
                    end
                end

                HS_BOOT_RD_DEVR: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch      <= 1'b1;
                        prim_launch_op   <= PRIM_WRITE;
                        prim_launch_byte <= DEV_ADDR_R;
                        hs_step_issued   <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        if (prim_ack_sample) begin
                            hs_state <= HS_IDLE;
                        end else begin
                            byte_idx <= 4'd0;
                            hs_state <= HS_BOOT_RD_BYTE;
                        end
                    end
                end

                HS_BOOT_RD_BYTE: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch      <= 1'b1;
                        prim_launch_op   <= PRIM_READ;
                        prim_launch_ack  <= (byte_idx == 4'd9); // last byte => NACK
                        hs_step_issued   <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        rd_buf[byte_idx] <= prim_rx_byte;
                        if (byte_idx == 4'd9)
                            hs_state <= HS_BOOT_RD_STOP;
                        else
                            byte_idx <= byte_idx + 1'b1;
                    end
                end

                HS_BOOT_RD_STOP: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch    <= 1'b1;
                        prim_launch_op <= PRIM_STOP;
                        hs_step_issued <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        hs_state <= HS_BOOT_CHECK;
                    end
                end

                HS_BOOT_CHECK: begin
                    checksum = 8'h00;
                    for (i = 0; i < 9; i = i + 1)
                        checksum = checksum ^ rd_buf[i];

                    if ((rd_buf[0] == MAGIC) &&
                        (rd_buf[1] == VERSION) &&
                        (checksum == rd_buf[9])) begin
                        load_state <= {
                            rd_buf[8][3:0],
                            rd_buf[7],
                            rd_buf[6],
                            rd_buf[5],
                            rd_buf[4],
                            rd_buf[3],
                            rd_buf[2]
                        };
                        load_valid <= 1'b1;
                    end

                    boot_done <= 1'b1;
                    hs_state  <= HS_IDLE;
                end

                // -------------------------
                // Idle / save sequence.
                // -------------------------
                HS_IDLE: begin
                    if (!boot_done)
                        hs_state <= HS_BOOT_SET_START;
                    else if (pending_save)
                        hs_state <= HS_SAVE_PREP;
                end

                HS_SAVE_PREP: begin
                    pending_save <= 1'b0; // consume latest snapshot

                    wr_buf[0] <= MAGIC;
                    wr_buf[1] <= VERSION;
                    wr_buf[2] <= latched_save_state[7:0];
                    wr_buf[3] <= latched_save_state[15:8];
                    wr_buf[4] <= latched_save_state[23:16];
                    wr_buf[5] <= latched_save_state[31:24];
                    wr_buf[6] <= latched_save_state[39:32];
                    wr_buf[7] <= latched_save_state[47:40];
                    wr_buf[8] <= {4'b0000, latched_save_state[51:48]};
                    wr_buf[9] <= MAGIC ^ VERSION ^
                                 latched_save_state[7:0] ^
                                 latched_save_state[15:8] ^
                                 latched_save_state[23:16] ^
                                 latched_save_state[31:24] ^
                                 latched_save_state[39:32] ^
                                 latched_save_state[47:40] ^
                                 {4'b0000, latched_save_state[51:48]};

                    byte_idx <= 4'd0;
                    hs_state <= HS_SAVE_START;
                end

                HS_SAVE_START: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch    <= 1'b1;
                        prim_launch_op <= PRIM_START;
                        hs_step_issued <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        hs_state <= HS_SAVE_DEVW;
                    end
                end

                HS_SAVE_DEVW: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch      <= 1'b1;
                        prim_launch_op   <= PRIM_WRITE;
                        prim_launch_byte <= DEV_ADDR_W;
                        hs_step_issued   <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        if (prim_ack_sample) begin
                            error    <= 1'b1;
                            hs_state <= HS_IDLE;
                        end else begin
                            hs_state <= HS_SAVE_ADDR;
                        end
                    end
                end

                HS_SAVE_ADDR: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch      <= 1'b1;
                        prim_launch_op   <= PRIM_WRITE;
                        // AT24C01C internal address is 7-bit, but we only use low 10 bytes.
                        prim_launch_byte <= {4'b0000, byte_idx};
                        hs_step_issued   <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        if (prim_ack_sample) begin
                            error    <= 1'b1;
                            hs_state <= HS_IDLE;
                        end else begin
                            hs_state <= HS_SAVE_DATA;
                        end
                    end
                end

                HS_SAVE_DATA: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch      <= 1'b1;
                        prim_launch_op   <= PRIM_WRITE;
                        prim_launch_byte <= wr_buf[byte_idx];
                        hs_step_issued   <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        if (prim_ack_sample) begin
                            error    <= 1'b1;
                            hs_state <= HS_IDLE;
                        end else begin
                            hs_state <= HS_SAVE_STOP;
                        end
                    end
                end

                HS_SAVE_STOP: begin
                    if (!hs_step_issued && !prim_busy) begin
                        prim_launch    <= 1'b1;
                        prim_launch_op <= PRIM_STOP;
                        hs_step_issued <= 1'b1;
                    end else if (hs_step_issued && prim_done) begin
                        hs_step_issued <= 1'b0;
                        wr_wait_cnt    <= 22'd0;
                        hs_state       <= HS_SAVE_WAIT;
                    end
                end

                HS_SAVE_WAIT: begin
                    if (wr_wait_cnt >= (TWR_WAIT_CYC - 1)) begin
                        if (byte_idx == 4'd9) begin
                            hs_state <= HS_IDLE;
                        end else begin
                            byte_idx <= byte_idx + 1'b1;
                            hs_state <= HS_SAVE_START;
                        end
                    end else begin
                        wr_wait_cnt <= wr_wait_cnt + 1'b1;
                    end
                end

                default: begin
                    hs_state <= HS_IDLE;
                end
            endcase

            // Latch latest requested save snapshot (placed after state machine so
            // a concurrent request cannot be overwritten by HS_SAVE_PREP).
            if (save_req) begin
                pending_save       <= 1'b1;
                latched_save_state <= save_state;
            end
        end
    end

endmodule
