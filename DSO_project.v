// -----------------------------------------------------------------------------
// DSO_project (Top)
// -----------------------------------------------------------------------------
// Purpose:
//   Top-level integration of the digital storage oscilloscope pipeline.
//   Key path: DDS sources -> sample_controller -> pingpong_buffer -> VGA stream.
//
// Clock Domains:
//   - clk_50m: control, DDS, sampling, write-side RAM.
//   - clk_25m: VGA timing, read-side RAM, renderer.
//
// Key Signals (English):
//   - capture_done: one capture frame has finished writing.
//   - capture_ready: sampling engine may start the next capture.
//   - frame_valid: read side currently has at least one committed frame.
//   - rd_frame_done: VGA consumed one full display frame.
//   - active_frame_start_addr: base address used by current display frame.
//   - dbg_status[63:0]: in-screen debug bus.
// -----------------------------------------------------------------------------
module DSO_project (
    input  wire        clk_50m,
    input  wire        clk_25m,
    input  wire        pll_locked,

    input  wire        sys_rst_n,

    input  wire        key_up_n,
    input  wire        key_down_n,
    input  wire        key_enter_n,
    input  wire        key_back_n,

    output wire        vga_hs,
    output wire        vga_vs,
    output wire [4:0]  vga_r,
    output wire [5:0]  vga_g,
    output wire [4:0]  vga_b,
    
    output wire        led_0
 
   // output wire        flash_cs_n,
    //output wire        flash_sck,
    //output wire        flash_mosi,
    //input  wire        flash_miso
);
// Physical key debouncing.
    wire [4:0] raw_keys = {sys_rst_n, key_up_n, key_down_n, key_enter_n, key_back_n};
    wire [4:0] clean_keys;
    key_debouncer #(
        .KEY_WIDTH(5),
        .WAIT_TIME(20'd1_000_000)
    ) u_key_debouncer (
        .clk_50m  (clk_50m),
        .key_raw  (raw_keys),
        .key_clean(clean_keys)
    );

    
 
    wire clean_rst_n       = clean_keys[4];
    wire base_rst_n;
    reg  [1:0] rst50_sync = 2'b00;
    reg  [1:0] rst25_sync = 2'b00;
    wire rst_n_50;
    wire rst_n_25;
    wire clean_key_up_n    = clean_keys[3];
    wire clean_key_down_n  = clean_keys[2];
    wire clean_key_enter_n = clean_keys[1];
    wire clean_key_back_n  = clean_keys[0];

    // Power-on reset stretcher:
    // ensure every core FSM sees a guaranteed low-reset window after PLL lock.
    reg [15:0] por_cnt_50 = 16'd0;
    reg        por_done_50 = 1'b0;
    always @(posedge clk_50m or negedge clean_rst_n) begin
        if (!clean_rst_n) begin
            por_cnt_50  <= 16'd0;
            por_done_50 <= 1'b0;
        end else if (!pll_locked) begin
            por_cnt_50  <= 16'd0;
            por_done_50 <= 1'b0;
        end else if (!por_done_50) begin
            por_cnt_50 <= por_cnt_50 + 16'd1;
            if (&por_cnt_50)
                por_done_50 <= 1'b1;
        end
    end

    assign base_rst_n = clean_rst_n & pll_locked & por_done_50;

    // Per-domain reset synchronizers: async assert, sync release.
    always @(posedge clk_50m or negedge base_rst_n) begin
        if (!base_rst_n)
            rst50_sync <= 2'b00;
        else
            rst50_sync <= {rst50_sync[0], 1'b1};
    end

    always @(posedge clk_25m or negedge base_rst_n) begin
        if (!base_rst_n)
            rst25_sync <= 2'b00;
        else
            rst25_sync <= {rst25_sync[0], 1'b1};
    end

    assign rst_n_50 = rst50_sync[1];
    assign rst_n_25 = rst25_sync[1];
    
// Key edge detection (active-low key press -> one-cycle pulse).
    reg [3:0] key_d0, key_d1;
    always @(posedge clk_50m or negedge rst_n_50) begin
        if (!rst_n_50) begin
            key_d0 <= 4'b1111;
            key_d1 <= 4'b1111;
        end else begin
            key_d0 <= {clean_key_up_n, clean_key_down_n, clean_key_enter_n, clean_key_back_n};
            key_d1 <= key_d0;
        end
    end

    wire key_up_p    = ~key_d0[3] & key_d1[3];
    wire key_down_p  = ~key_d0[2] & key_d1[2];
    wire key_enter_p = ~key_d0[1] & key_d1[1];
    wire key_back_p  = ~key_d0[0] & key_d1[0];

    
// HMI / configuration control.
    wire [31:0] dds_freq_a, dds_freq_b, dds_freq_c, dds_freq_d, dds_freq_e;
    wire [1:0]  dds_type_a, dds_type_b, dds_type_c, dds_type_d, dds_type_e;
    wire [7:0]  dds_phase_a, dds_phase_b, dds_phase_c, dds_phase_d, dds_phase_e;
    
    wire [2:0]  sel_trig, sel_ch1, sel_ch2, sel_ch3, sel_ch4;
    wire        trig_mode, trig_edge;
    wire [7:0]  trig_level;
    wire [31:0] sample_div;
    
    wire        flash_write_req, flash_read_req;
    wire [1:0]  flash_ch_sel;

    wire [3:0]  ui_page, ui_cursor;
    wire [2:0]  view_ch_sel;
    wire        ui_curr_edit_mode;
    wire [3:0]  ui_curr_edit_value;
    wire [2:0]  ui_active_src_sel;



    hmi_controller u_hmi_controller (
        .clk_50m        (clk_50m),
        .rst_n          (rst_n_50),

        .key_up_p       (key_up_p),
        .key_down_p     (key_down_p),
        .key_enter_p    (key_enter_p),
        .key_back_p     (key_back_p),

        .dds_freq_a     (dds_freq_a),
        .dds_freq_b     (dds_freq_b),
        .dds_freq_c     (dds_freq_c),
        .dds_freq_d     (dds_freq_d),
        .dds_freq_e     (dds_freq_e),
        .dds_type_a     (dds_type_a),
        .dds_type_b     (dds_type_b),
        .dds_type_c     (dds_type_c),
        .dds_type_d     (dds_type_d),
        .dds_type_e     (dds_type_e),
        .dds_phase_a    (dds_phase_a),
        .dds_phase_b    (dds_phase_b),
        .dds_phase_c    (dds_phase_c),
        .dds_phase_d    (dds_phase_d),
        .dds_phase_e    (dds_phase_e),

        .sel_trig       (sel_trig),
        .sel_ch1        (sel_ch1),
        .sel_ch2        (sel_ch2),
        .sel_ch3        (sel_ch3),
        .sel_ch4        (sel_ch4),
        .trig_mode      (trig_mode),
        .trig_edge      (trig_edge),
        .trig_level     (trig_level),
        .sample_div     (sample_div),
       
        
        .flash_write_req(flash_write_req),
        .flash_ch_sel   (flash_ch_sel),
        .flash_read_req (flash_read_req),

        .ui_page        (ui_page),
        .ui_cursor      (ui_cursor),
        .ui_curr_edit_mode  (ui_curr_edit_mode),
        .ui_curr_edit_value  (ui_curr_edit_value),
        .ui_active_src_sel   (ui_active_src_sel),
        .view_ch_sel    (view_ch_sel)
    );
 



// DDS waveform generators.
    wire [7:0] sig_a, sig_b, sig_c, sig_d, sig_e;
    dds_generator u_dds_a (.clk(clk_50m), .rst_n(rst_n_50), .freq_word(dds_freq_a), .phase_offset(dds_phase_a), .wave_type(dds_type_a), .wave_data(sig_a));
    dds_generator u_dds_b (.clk(clk_50m), .rst_n(rst_n_50), .freq_word(dds_freq_b), .phase_offset(dds_phase_b), .wave_type(dds_type_b), .wave_data(sig_b));
    dds_generator u_dds_c (.clk(clk_50m), .rst_n(rst_n_50), .freq_word(dds_freq_c), .phase_offset(dds_phase_c), .wave_type(dds_type_c), .wave_data(sig_c));
    dds_generator u_dds_d (.clk(clk_50m), .rst_n(rst_n_50), .freq_word(dds_freq_d), .phase_offset(dds_phase_d), .wave_type(dds_type_d), .wave_data(sig_d));
    dds_generator u_dds_e (.clk(clk_50m), .rst_n(rst_n_50), .freq_word(dds_freq_e), .phase_offset(dds_phase_e), .wave_type(dds_type_e), .wave_data(sig_e));
// Sampling controller.
    wire              ram_we;
    wire [9:0]        ram_waddr;
    wire [7:0]        ram_wdata_ch1, ram_wdata_ch2, ram_wdata_ch3, ram_wdata_ch4;
    wire              capture_done;
    wire [9:0]        frame_start_addr;
    wire              capture_ready;

    sample_controller u_sample_controller (
        .clk             (clk_50m),
        .rst_n           (rst_n_50),

        .in_a            (sig_a),
        .in_b            (sig_b),
        .in_c            (sig_c),
        .in_d            (sig_d),
        .in_e            (sig_e),

        .sel_trig        (sel_trig),
        .sel_ch1         (sel_ch1),
        .sel_ch2         (sel_ch2),
        .sel_ch3         (sel_ch3),
        .sel_ch4         (sel_ch4),
        .trig_mode       (trig_mode),
        .trig_edge       (trig_edge),
        .trig_level      (trig_level),
        .sample_div      (sample_div),

        .rearm           (capture_ready),
        .capture_done    (capture_done),
        .frame_start_addr(frame_start_addr),
        .ram_we          (ram_we),
        
        .ram_waddr       (ram_waddr),
        .ram_wdata_ch1   (ram_wdata_ch1),
        .ram_wdata_ch2   (ram_wdata_ch2),
        .ram_wdata_ch3   (ram_wdata_ch3),
        .ram_wdata_ch4   (ram_wdata_ch4)
    );
   
    // Ping-pong frame buffer.
    //wire [9:0] vga_wave_phys_raddr = active_frame_start_addr + vga_wave_logical_raddr;
    wire [9:0] active_frame_start_addr;
    wire       frame_valid;
    wire [7:0] vga_rdata_ch1, vga_rdata_ch2, vga_rdata_ch3, vga_rdata_ch4;
    wire       rd_frame_done;
    wire       [9:0] vga_raddr;
    wire       overflow;
    wire [63:0] dbg_status;
    wire [15:0] pp_dbg_bus;
    wire [3:0]  pp_read_tap;
    


    pingpong_buffer u_pingpong_buffer (
        .rst_n_write             (rst_n_50),
        .rst_n_read              (rst_n_25),

        .clk_write               (clk_50m),
        .waddr                   (ram_waddr),
        .wdata_ch1               (ram_wdata_ch1),
        .wdata_ch2               (ram_wdata_ch2),
        .wdata_ch3               (ram_wdata_ch3),
        .wdata_ch4               (ram_wdata_ch4),

        .clk_read                (clk_25m),
        .raddr                   (vga_raddr),
        .rdata_ch1               (vga_rdata_ch1),
        .rdata_ch2               (vga_rdata_ch2),
        .rdata_ch3               (vga_rdata_ch3),
        .rdata_ch4               (vga_rdata_ch4),

        .capture_done            (capture_done),
        .we                      (ram_we),
        .rd_frame_done    (rd_frame_done),    // read-domain pulse: one display frame consumed
        .overflow          (overflow),
        .frame_start_addr     (frame_start_addr),
        .active_frame_start_addr     (active_frame_start_addr),
        .capture_ready    (capture_ready),
        .frame_valid      (frame_valid),
        .dbg_bus          (pp_dbg_bus),
        .dbg_read_tap     (pp_read_tap)
        

    );

    // In-screen debug status (64 bits, no external debug pins required):
    // [0]   rst_n_25
    // [1]   rst_n_50 synced into 25m domain
    // [2]   capture_ready synced into 25m domain
    // [3]   frame_valid
    // [4]   overflow synced into 25m domain
    // [5]   rd_frame_done seen since reset
    // [6]   non-zero sample seen since reset
    // [7]   recent capture_done pulse (stretched in 25m domain)
    // [8]   wr_bank_sel
    // [9]   active_bank_sync2_wr
    // [10]  writing_active_bank
    // [11]  capture_active_wr
    // [12]  bank0_commit_tog_wr
    // [13]  bank1_commit_tog_wr
    // [14]  active_bank_rd
    // [15]  pending_valid_rd
    // [16]  pending_bank_rd
    // [17]  frame_valid (from pingpong read domain)
    // [18]  we_seen_wr (sticky)
    // [19]  capture_done_seen_wr (sticky)
    // [20]  wrote_active_bank_seen_wr (sticky)
    // [21]  overrun_seen_wr (sticky)
    // [22]  bank0_we_a_seen_wr (sticky, real BRAM write hit)
    // [23]  bank1_we_a_seen_wr (sticky, real BRAM write hit)
    // [24]  (vga_rdata_ch1 != 0)
    // [25]  (vga_rdata_ch2 != 0)
    // [26]  (vga_rdata_ch3 != 0)
    // [27]  (vga_rdata_ch4 != 0)
    // [28]  (ram_we && ram_wdata_ch1 != 0), synced to 25m
    // [29]  (ram_we && ram_wdata_ch2 != 0), synced to 25m
    // [30]  (ram_we && ram_wdata_ch3 != 0), synced to 25m
    // [31]  (ram_we && ram_wdata_ch4 != 0), synced to 25m
    // [32]  active_bank_rd live mirror (from pp bit6)
    // [33]  pending_valid_rd live mirror (from pp bit7)
    // [34]  rd_frame_done pulse (real-time)
    // [35]  raddr changed this cycle (real-time)
    // [36]  raddr_changed_seen since reset (sticky)
    // [37]  raddr_wrap_seen since reset (sticky)
    // [38]  (vga_raddr != 0)
    // [39]  (active_frame_start_addr != 0)
    // [40..49] vga_raddr[9:0] (real-time)
    // [50..59] active_frame_start_addr[9:0] (real-time)
    // [60]  bank0 raw read data non-zero (from pingpong bank0_rdata_b)
    // [61]  bank1 raw read data non-zero (from pingpong bank1_rdata_b)
    // [62]  active_bank raw read data non-zero
    // [63]  final muxed rdata_ch* non-zero
    reg capdone_tog_50;
    always @(posedge clk_50m or negedge rst_n_50) begin
        if (!rst_n_50)
            capdone_tog_50 <= 1'b0;
        else if (capture_done)
            capdone_tog_50 <= ~capdone_tog_50;
    end

    reg rst50_sync1_25, rst50_sync2_25;
    reg capready_sync1_25, capready_sync2_25;
    reg overflow_sync1_25, overflow_sync2_25;
    reg capdone_sync1_25, capdone_sync2_25, capdone_sync2_d_25;
    reg [15:0] pp_dbg_sync1_25, pp_dbg_sync2_25;
    reg [3:0] wr_nz_sync1_25, wr_nz_sync2_25;
    reg [21:0] capdone_hold_25;
    reg rd_seen_25;
    reg data_seen_25;
    reg [9:0] prev_vga_raddr_25;
    reg raddr_changed_seen_25;
    reg raddr_wrap_seen_25;
    wire capdone_pulse_25 = capdone_sync2_25 ^ capdone_sync2_d_25;

    always @(posedge clk_25m or negedge rst_n_25) begin
        if (!rst_n_25) begin
            rst50_sync1_25     <= 1'b0;
            rst50_sync2_25     <= 1'b0;
            capready_sync1_25  <= 1'b0;
            capready_sync2_25  <= 1'b0;
            overflow_sync1_25  <= 1'b0;
            overflow_sync2_25  <= 1'b0;
            capdone_sync1_25   <= 1'b0;
            capdone_sync2_25   <= 1'b0;
            capdone_sync2_d_25 <= 1'b0;
            pp_dbg_sync1_25    <= 16'd0;
            pp_dbg_sync2_25    <= 16'd0;
            wr_nz_sync1_25     <= 4'd0;
            wr_nz_sync2_25     <= 4'd0;
            capdone_hold_25    <= 22'd0;
            rd_seen_25         <= 1'b0;
            data_seen_25       <= 1'b0;
            prev_vga_raddr_25  <= 10'd0;
            raddr_changed_seen_25 <= 1'b0;
            raddr_wrap_seen_25 <= 1'b0;
        end else begin
            rst50_sync1_25     <= rst_n_50;
            rst50_sync2_25     <= rst50_sync1_25;
            capready_sync1_25  <= capture_ready;
            capready_sync2_25  <= capready_sync1_25;
            overflow_sync1_25  <= overflow;
            overflow_sync2_25  <= overflow_sync1_25;
            capdone_sync1_25   <= capdone_tog_50;
            capdone_sync2_25   <= capdone_sync1_25;
            capdone_sync2_d_25 <= capdone_sync2_25;
            pp_dbg_sync1_25    <= pp_dbg_bus;
            pp_dbg_sync2_25    <= pp_dbg_sync1_25;
            wr_nz_sync1_25     <= {
                (ram_we && (ram_wdata_ch4 != 8'd0)),
                (ram_we && (ram_wdata_ch3 != 8'd0)),
                (ram_we && (ram_wdata_ch2 != 8'd0)),
                (ram_we && (ram_wdata_ch1 != 8'd0))
            };
            wr_nz_sync2_25     <= wr_nz_sync1_25;

            if (capdone_pulse_25)
                capdone_hold_25 <= 22'h3fffff;
            else if (capdone_hold_25 != 22'd0)
                capdone_hold_25 <= capdone_hold_25 - 1'b1;

            if (rd_frame_done)
                rd_seen_25 <= 1'b1;

            if (frame_valid &&
                ((vga_rdata_ch1 != 8'd0) || (vga_rdata_ch2 != 8'd0) ||
                 (vga_rdata_ch3 != 8'd0) || (vga_rdata_ch4 != 8'd0)))
                data_seen_25 <= 1'b1;

            if (vga_raddr != prev_vga_raddr_25)
                raddr_changed_seen_25 <= 1'b1;
            if (vga_raddr < prev_vga_raddr_25)
                raddr_wrap_seen_25 <= 1'b1;
            prev_vga_raddr_25 <= vga_raddr;
        end
    end

    wire [7:0] dbg_status_base;
    wire raddr_changed_rt = (vga_raddr != prev_vga_raddr_25);
    wire ch1_nz_rt = (vga_rdata_ch1 != 8'd0);
    wire ch2_nz_rt = (vga_rdata_ch2 != 8'd0);
    wire ch3_nz_rt = (vga_rdata_ch3 != 8'd0);
    wire ch4_nz_rt = (vga_rdata_ch4 != 8'd0);
    assign dbg_status_base = {
        (capdone_hold_25 != 22'd0),
        data_seen_25,
        rd_seen_25,
        overflow_sync2_25,
        frame_valid,
        capready_sync2_25,
        rst50_sync2_25,
        rst_n_25
    };
    assign dbg_status = {
        pp_read_tap,
        active_frame_start_addr,
        vga_raddr,
        (|active_frame_start_addr),
        (|vga_raddr),
        raddr_wrap_seen_25,
        raddr_changed_seen_25,
        raddr_changed_rt,
        rd_frame_done,
        pp_dbg_sync2_25[7],
        pp_dbg_sync2_25[6],
        wr_nz_sync2_25,
        ch4_nz_rt,
        ch3_nz_rt,
        ch2_nz_rt,
        ch1_nz_rt,
        pp_dbg_sync2_25,
        dbg_status_base
    };

   assign  led_0 = ~overflow;
   wire [15:0] rgb_565;
   assign vga_r     = rgb_565[15:11];
   assign vga_g     = rgb_565[10:5];
   assign vga_b     = rgb_565[4:0];
   
    VGA_top u_VGA_top(/*AUTOINST*/
		      // Outputs
		      .raddr		(vga_raddr),
		      .rd_frame_done	(rd_frame_done),
		      .hsync		(vga_hs),
		      .vsync		(vga_vs),
		      .rgb565		(rgb_565),
		      // Inputs
		      .clk_25m		(clk_25m),
		      .rst_n		(rst_n_25),
		      .frame_valid	(frame_valid),
		      .active_frame_start_addr(active_frame_start_addr),
		      .rdata_ch1	(vga_rdata_ch1),
		      .rdata_ch2	(vga_rdata_ch2),
		      .rdata_ch3	(vga_rdata_ch3),
		      .rdata_ch4	(vga_rdata_ch4),
		      .dbg_status       (dbg_status)

		      //.ui_page		(ui_page),
		      //.ui_cursor	(ui_cursor),
		      //.ui_curr_edit_mode(ui_curr_edit_mode),
		      //.ui_curr_edit_valu(ui_curr_edit_valu),
		      //.ui_active_src_sel(ui_active_src_sel),
		      //.view_ch_sel	(view_ch_sel)
          );
   
endmodule
