// ====================================================================
//
//  NEC u765 FDC
//
//  Copyright (C) 2017 Gyorgy Szombathelyi <gyurco@freemail.hu>
//
//  Updated for PCW disks and timing, including interrupt, ndma mode and TC handling
//  Changes Copyright (C) 2020 Stephen Eddy
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

//TODO:
//GAP, CRC generation
//WRITE DELETE should write the Deleted Address Mark to the SectorInfo
//real FORMAT (but this would require squeezing/expanding the image file)

// For accurate head stepping rate, set CYCLES to cycles/ms
// 4MHz = 4000 (default).  If a faster clock is fed in, this will just speed up the simulation in line
// SPECCY_SPEEDLOCK_HACK: auto mess-up weak sector on C0H0S2


module u765 #(
    parameter CYCLES = 20'd4000,
    SPECCY_SPEEDLOCK_HACK = 0
) (
    input  wire        clk_sys,    // sys clock
    input  wire        ce,         // chip enable
    input  wire        reset,      // reset
    input  wire  [1:0] ready,      // disk is inserted in MiST(er)
    input  wire  [1:0] motor,      // drive motor
    input  wire  [1:0] available,  // drive available (fake ready signal for SENSE DRIVE command)
    input  wire        a0,
    input  wire        nRD,        // i/o read
    input  wire        nWR,        // i/o write
    input  wire  [7:0] din,        // i/o data in
    output logic [7:0] dout,       // i/o data out

    input  wire        tc,            // terminal count (terminate)
    output logic       int_out,       // Output interrupt line
    input  wire  [1:0] density,       // CF2 = 0, CF2DD = 1
    output logic       activity_led,  // Activity LED
    input  wire        fast,          // Fast mode   
    output wire  [ 1:0] prepare,
    input  wire  [ 1:0] img_mounted,   // signaling that new image has been mounted
    input  wire  [ 1:0] img_wp,        // write protect. latched at img_mounted
    input  wire  [31:0] img_size,      // size of image in bytes
    output logic [31:0] sd_lba,
    output logic [ 1:0] sd_rd,
    output logic [ 1:0] sd_wr,
    input  wire  [ 1:0] sd_ack,
    input  wire  [ 8:0] sd_buff_addr,
    input  wire  [ 7:0] sd_buff_dout,
    output logic [ 7:0] sd_buff_din,
    input  wire         sd_buff_wr,

    output logic [7:0] old_state
);

  //localparam OVERRUN_TIMEOUT = 26'd35000000;
  // This should equal 13 us
  localparam OVERRUN_TIMEOUT = CYCLES * 10'd100;  // 13us seconds assuming base clock of 4Mhz
  // Sector time - We are going to fix this to 9 sectors per track for PCW
  localparam SECTOR_TIME = ((CYCLES * 20'd200) / 20'd9) / 20'd4;  // SECTOR time for timing of disk speed.

  localparam UPD765_MAIN_D0B = 0;
  localparam UPD765_MAIN_D1B = 1;
  localparam UPD765_MAIN_D2B = 2;
  localparam UPD765_MAIN_D3B = 3;
  localparam UPD765_MAIN_CB = 4;
  localparam UPD765_MAIN_EXM = 5;
  localparam UPD765_MAIN_DIO = 6;
  localparam UPD765_MAIN_RQM = 7;

  // Disk densities
  localparam CF2 = 1'b0;
  localparam CF2DD = 1'b1;

  localparam UPD765_SD_BUFF_TRACKINFO = 1'd0;
  localparam UPD765_SD_BUFF_SECTOR = 1'd1;

  typedef enum bit [6:0] {
  COMMAND_IDLE,                  // 00 - Command processor idle state
  COMMAND_READ_TRACK,            // 01 - Read entire track
  COMMAND_WRITE_DELETED_DATA,    // 02 - Write data with deleted address mark
  COMMAND_WRITE_DATA,            // 03 - Normal write data
  COMMAND_READ_DELETED_DATA,     // 04 - Read data with deleted address mark
  COMMAND_READ_DATA,             // 05 - Normal read data
  COMMAND_RW_DATA_EXEC,          // 06 - Start of read/write execution
  COMMAND_RW_DATA_EXEC1,         // 07 - R/W execution step 1
  COMMAND_RW_DATA_EXEC2,         // 08 - R/W execution step 2
  COMMAND_RW_DATA_EXEC3,         // 09 - R/W execution step 3
  COMMAND_RW_DATA_EXEC4,         // 0A - R/W execution step 4
  COMMAND_RW_DATA_EXEC5,         // 0B - R/W execution step 5
  COMMAND_RW_DATA_WAIT_SECTOR,   // 0C - Wait for correct sector position
  COMMAND_RW_DATA_EXEC_WEAK,     // 0D - Handle weak sector processing
  COMMAND_RW_DATA_EXEC6,         // 0E - R/W execution step 6
  COMMAND_RW_DATA_EXEC7,         // 0F - R/W execution step 7
  COMMAND_RW_DATA_EXEC8,         // 10 - R/W execution step 8
  COMMAND_RW_DATA_SCAN_COMPARE,  // 11 - Compare data during SCAN commands
  COMMAND_READ_ID,               // 12 - Read ID command
  COMMAND_READ_ID1,              // 13 - Read ID step 1
  COMMAND_READ_ID2,              // 14 - Read ID step 2
  COMMAND_READ_ID_EXEC1,         // 15 - Read ID execution step 1
  COMMAND_READ_ID_WAIT_SECTOR,   // 16 - Wait for sector during Read ID
  COMMAND_READ_ID_EXEC2,         // 17 - Read ID execution step 2
  COMMAND_FORMAT_TRACK,          // 18 - Format track command
  COMMAND_FORMAT_TRACK1,         // 19 - Format track step 1
  COMMAND_FORMAT_TRACK2,         // 1A - Format track step 2
  COMMAND_FORMAT_TRACK3,         // 1B - Format track step 3
  COMMAND_FORMAT_TRACK4,         // 1C - Format track step 4
  COMMAND_FORMAT_TRACK5,         // 1D - Format track step 5
  COMMAND_FORMAT_TRACK6,         // 1E - Format track step 6
  COMMAND_FORMAT_TRACK7,         // 1F - Format track step 7
  COMMAND_FORMAT_TRACK8,         // 20 - Format track step 8
  COMMAND_SCAN_EQUAL,            // 21 - Scan Equal command
  COMMAND_SCAN_LOW_OR_EQUAL,     // 22 - Scan Low or Equal command
  COMMAND_SCAN_HIGH_OR_EQUAL,    // 23 - Scan High or Equal command
  COMMAND_RECALIBRATE,           // 24 - Recalibrate (seek to track 0)
  COMMAND_SENSE_INTERRUPT_STATUS,   // 25 - Sense interrupt status
  COMMAND_SENSE_INTERRUPT_STATUS1,  // 26 - Sense interrupt status step 1
  COMMAND_SENSE_INTERRUPT_STATUS2,  // 27 - Sense interrupt status step 2
  COMMAND_SPECIFY,               // 28 - Specify parameters
  COMMAND_SPECIFY_WR,            // 29 - Specify parameters write
  COMMAND_SENSE_DRIVE_STATUS,    // 2A - Sense drive status
  COMMAND_SENSE_DRIVE_STATUS_RD, // 2B - Sense drive status read
  COMMAND_SEEK,                  // 2C - Seek to track
  COMMAND_SEEK_EXEC1,            // 2D - Seek execution step 1
  COMMAND_SETUP,                 // 2E - Common command parameter setup
  COMMAND_READ_RESULTS,          // 2F - Read command result bytes
  COMMAND_INVALID,               // 30 - Invalid command handling
  COMMAND_INVALID1,              // 31 - Invalid command step 1
  COMMAND_RELOAD_TRACKINFO,      // 32 - Reload track information
  COMMAND_RELOAD_TRACKINFO1,     // 33 - Reload track information step 1
  COMMAND_RELOAD_TRACKINFO2,     // 34 - Reload track information step 2
  COMMAND_RELOAD_TRACKINFO3,     // 35 - Reload track information step 3
  COMMAND_SCAN_SETUP,            // 36 - Setup for SCAN commands
  COMMAND_SETUP_VALIDATION,      // 37 - Validate command parameters
  COMMAND_RESET,                  // 38 - Reset controller state
  COMMAND_SCAN_EXEC1,
  COMMAND_SCAN_EXEC2,
  COMMAND_SCAN_EXEC3, 
  COMMAND_SCAN_EXEC4,
  COMMAND_SCAN_READ_SECTOR,
  COMMAND_SCAN_COMPARE,
  COMMAND_SCAN_NEXT,
  COMMAND_FAKE
} state_t;

typedef enum bit [1:0] {
  PHASE_COMMAND,   // Command input phase
  PHASE_EXECUTE,   // Command execution phase
  PHASE_RESPONSE   // Result phase
} phase_t;

  // sector/trackinfo buffers
  logic [7:0] buff_data_in  /* synthesis keep */;
  logic [7:0] buff_data_out;
  logic [8:0] buff_addr;
  logic buff_wr, buff_wait;
  logic sd_buff_type;
  logic hds, ds0;

  u765_dpram sbuf (
      .clock(clk_sys),
      // SD card read / write access
      .address_a({ds0, sd_buff_type, hds, sd_buff_addr}),
      .data_a(sd_buff_dout),
      .wren_a(sd_buff_wr & sd_ack[ds0]),
      .q_a(sd_buff_din),
      // FDC module read write access for processor
      .address_b({ds0, sd_buff_type, hds, buff_addr}),
      .data_b(buff_data_out),
      .wren_b(buff_wr),
      .q_b(buff_data_in)
  );

  //track offset buffer
  //single port buffer in RAM
  logic [15:0] image_track_offsets          [1024];  //offset of tracks * 256 * 2 drives
  reg   [ 8:0] image_track_offsets_addr = 0;
  reg          image_track_offsets_wr;
  reg [15:0] image_track_offsets_out, image_track_offsets_in;

  always @(posedge clk_sys) begin
    if (image_track_offsets_wr) begin
      image_track_offsets[{ds0, image_track_offsets_addr}] <= image_track_offsets_out;
      image_track_offsets_in <= image_track_offsets_out;
    end else begin
      image_track_offsets_in <= image_track_offsets[{ds0, image_track_offsets_addr}];
    end
  end

  logic rd;
  assign rd = nWR & ~nRD;
  logic wr;
  assign wr = ~nWR & nRD;
  logic [7:0] i_total_sectors;

  phase_t phase;

  reg [7:0] m_status;  //main status register
  reg [7:0] m_data;  //data register
  reg int_state[2];  // interrupt states for both drives

  logic ndma_mode = 1'b1;
  state_t last_state;

  reg   [1:0] i_scan_mode[2];  // 0=normal, 1=equal, 2=low_or_equal, 3=high_or_equal
  reg         i_scan_match;  // Indica si se encontr?? una coincidencia durante el escaneo
  reg   [7:0] i_stp;  // Step (incremento de sectores a saltar)

  reg [1:0] image_ready;

  assign int_out = int_state[0] | int_state[1];
  assign dout = a0 ? m_data : m_status;
  assign old_state = last_state;
  assign activity_led = (phase == PHASE_EXECUTE);
  assign prepare = image_ready;





  always @(posedge clk_sys) begin

    //prefix internal CE protected registers with i_, so it's easier to write constraints

    //per-drive data
    reg [31:0] image_size[2];
    reg [7:0] image_tracks[2];
    reg image_sides[2];  //1 side - 0, 2 sides - 1
    reg [1:0] image_wp;
    reg image_trackinfo_dirty[2];
    reg image_edsk[2];  //DSK - 0, EDSK - 1
    reg [1:0] image_scan_state[2] ;
    reg [1:0] image_density;
    reg [7:0] i_current_track_sectors[2][2] /* synthesis keep */;  //number of sectors on the current track /head/drive
    reg [7:0] i_current_sector_pos[2][2] /* synthesis keep */; //sector where the head currently positioned
    reg [19:0] i_steptimer[2], i_rpm_timer[2][2];
    reg [3:0] i_step_state[2];  //counting cycles_time for steptimer

    reg [7:0] ncn[2];  //new cylinder number
    reg [7:0] pcn[2];  //present cylinder number
    reg [2:0] next_weak_sector[2];
    reg [1:0] seek_state[2];

    reg old_wr, old_rd;
    reg [ 7:0] i_track_size;
    reg [31:0] i_seek_pos;
    reg [7:0] i_sector_c, i_sector_h, i_sector_r, i_sector_n;
    reg [7:0] i_sector_st1, i_sector_st2;
    reg [15:0] i_sector_size;
    reg [7:0] i_current_sector;
    reg i_scanning;
    reg [2:0] i_weak_sector;
    reg [15:0] i_bytes_to_read;
    reg [2:0] i_substate;
    reg [1:0] old_mounted;
    reg [1:0] old_ready;
    reg [15:0] i_track_offset;
    reg [5:0] ack;
    reg sd_busy;
    reg [19:0] i_timeout;
    reg [7:0] i_head_timer;
    reg i_rtrack, i_write, i_rw_deleted;
    reg [7:0] status[4];  //st0-3
    state_t state;
    state_t i_command;
    reg i_current_drive, i_scan_lock;
    reg   [3:0] i_srt;  //stepping rate
    reg   [3:0] i_hut;  //head unload time
    reg   [6:0] i_hlt;  //head load time
    reg   [7:0] i_c;
    reg   [7:0] i_h;
    reg   [7:0] i_r;
    reg   [7:0] i_n;
    reg   [7:0] i_eot;
    //reg [7:0] i_gpl;
    reg   [7:0] i_dtl;
    reg   [7:0] i_sc;
    //reg [7:0] i_d;
    reg         i_bc;  //bad cylinder
    reg         old_hds;
    reg         old_tc;
    logic [7:0] tmp_ncn;

    reg         i_mt;
    //reg i_mfm;
    reg         i_sk;

   

    buff_wait <= 0;
    i_total_sectors = i_current_track_sectors[ds0][hds];

    //new image mounted
    for (int i = 0; i < 2; i++) begin
      old_mounted[i] <= img_mounted[i];
      old_ready[i]   <= ready[i];
      if (~old_mounted[i] & img_mounted[i]) begin
        image_wp[i] <= img_wp[i];
        image_size[i] <= img_size;
        image_scan_state[i] <= |img_size;  //hacky
        image_ready[i] <= 0;
        image_density[i] <= (img_size > 250000) ? CF2DD : CF2;  // very hacky
        //int_state[i] <= 1;
        seek_state[i] <= 0;
        next_weak_sector[i] <= 0;
        i_current_sector_pos[i] <= '{0, 0};
      end
    end


    //Process the image file
    if (ce) begin
      i_current_drive <= ~i_current_drive;
      case (image_scan_state[i_current_drive])
        0: ;  //no new image
        1:  //read the first 512 byte
        if (~sd_busy & ~i_scan_lock & state == COMMAND_IDLE) begin
          sd_buff_type <= UPD765_SD_BUFF_SECTOR;
          i_scan_lock <= 1;
          ds0 <= i_current_drive;
          sd_rd[i_current_drive] <= 1;
          sd_lba <= 0;
          sd_busy <= 1;
          i_track_offset <= 16'h1;  //offset 100h
          image_track_offsets_addr <= 0;
          buff_addr <= 0;
          buff_wait <= 1;
          image_scan_state[i_current_drive] <= 2;
        end
        2:  //process the header - Update all the image track offsets for every track
        if (~sd_busy & ~buff_wait) begin
          if (buff_addr == 0) begin
            if (buff_data_in == "E") image_edsk[i_current_drive] <= 1;
            else if (buff_data_in == "M") image_edsk[i_current_drive] <= 0;
            else begin
              image_ready[i_current_drive] <= 0;
              image_scan_state[i_current_drive] <= 0;
              i_scan_lock <= 0;
            end
          end else if (buff_addr == 9'h30) image_tracks[i_current_drive] <= buff_data_in;
          else if (buff_addr == 9'h31) image_sides[i_current_drive] <= buff_data_in[1];
          else if (buff_addr == 9'h33) i_track_size <= buff_data_in;
          else if (buff_addr >= 9'h34) begin
            if (image_track_offsets_addr[8:1] != image_tracks[i_current_drive]) begin
              image_track_offsets_wr <= 1;
              if (image_edsk[i_current_drive]) begin
                image_track_offsets_out <= buff_data_in ? i_track_offset : 16'd0;
                i_track_offset <= i_track_offset + buff_data_in;
              end else begin
                image_track_offsets_out <= i_track_offset;
                i_track_offset <= i_track_offset + i_track_size;
              end
              image_scan_state[i_current_drive] <= 3;
            end else begin
              $display("*** Setting image_ready[%d]=1, tracks=%d, sides=%d", i_current_drive,
                       image_tracks[i_current_drive], image_sides[i_current_drive]);
              image_ready[i_current_drive] <= 1;
              image_scan_state[i_current_drive] <= 0;
              image_trackinfo_dirty[i_current_drive] <= 1;
              i_scan_lock <= 0;
            end
          end
          buff_addr <= buff_addr + 1'd1;
          buff_wait <= 1;
        end
        3: begin
          image_track_offsets_wr <= 0;
          image_track_offsets_addr <= image_track_offsets_addr + { ~image_sides[i_current_drive], image_sides[i_current_drive] };
          image_scan_state[i_current_drive] <= 2;
        end
      endcase
    end

    //the FDC
    if (reset) begin
      old_tc <= 1'b0;
      m_status <= 8'h80;
      state <= COMMAND_IDLE;
      last_state <= COMMAND_IDLE;
      phase <= PHASE_COMMAND;
      status[0] <= 0;
      status[1] <= 0;
      status[2] <= 0;
      status[3] <= 0;
      ncn <= '{0, 0};
      pcn <= '{0, 0};
      int_state <= '{0, 0};
      seek_state <= '{0, 0};
      image_trackinfo_dirty <= '{1, 1};
      {ack, sd_busy} <= 0;
      sd_rd <= 0;
      sd_wr <= 0;
      sd_busy <= 0;
      image_track_offsets_wr <= 0;
      //restart "mounting" of image(s)
      if (image_scan_state[0]) image_scan_state[0] <= 1;
      if (image_scan_state[1]) image_scan_state[1] <= 1;
      i_scan_lock <= 0;
      i_srt <= 4;
      ndma_mode <= 1'b1;
      i_scan_mode <= '{0, 0};  // Inicializacion del modo de escaneo
    end else if (ce) begin

      ack <= {ack[4:0], sd_ack[ds0]};
      if (ack[5:4] == 'b01) begin
        sd_rd <= 0;
        sd_wr <= 0;
      end
      if (ack[5:4] == 'b10) sd_busy <= 0;

      old_wr <= wr;
      old_rd <= rd;

      //seek (track stepping - step 0 = not stepping)
      case (seek_state[i_current_drive])
        0: ;  //no seek in progress
        1:
        if (pcn[i_current_drive] == ncn[i_current_drive]) begin
          int_state[i_current_drive]  <= 1;
          seek_state[i_current_drive] <= 0;
          //i_current_sector <= 1'd1;
        end else begin
          image_trackinfo_dirty[i_current_drive] <= 1;
          if (fast) begin
            pcn[i_current_drive] <= ncn[i_current_drive];
          end else begin
            if (pcn[i_current_drive] > ncn[i_current_drive])
              pcn[i_current_drive] <= pcn[i_current_drive] - 1'd1;
            if (pcn[i_current_drive] < ncn[i_current_drive])
              pcn[i_current_drive] <= pcn[i_current_drive] + 1'd1;
            i_step_state[i_current_drive] <= i_srt;
            i_steptimer[i_current_drive]  <= CYCLES;
            seek_state[i_current_drive]   <= 2;
          end
        end
        2:
        if (i_steptimer[i_current_drive]) begin
          i_steptimer[i_current_drive] <= i_steptimer[i_current_drive] - 1'd1;
        end else if (~&i_step_state[i_current_drive]) begin
          i_step_state[i_current_drive] <= i_step_state[i_current_drive] + 1'd1;
          i_steptimer[i_current_drive]  <= CYCLES;
        end else begin
          seek_state[i_current_drive] <= 1;
        end
      endcase

      //disk rotation
      if (motor[i_current_drive]) begin
        for (int i = 0; i < 2; i++) begin
          if (i_rpm_timer[i_current_drive][i] >= SECTOR_TIME) begin
            // i_current_sector_pos is physical sector number on track (e.g. 1,2,3,etc)
            i_current_sector_pos[i_current_drive][i] <=
					i_current_sector_pos[i_current_drive][i] == i_current_track_sectors[i_current_drive][i] - 1'd1 ?
						8'd0 : i_current_sector_pos[i_current_drive][i] + 1'd1;
            i_rpm_timer[i_current_drive][i] <= 0;
          end else begin
            if(state != COMMAND_RW_DATA_EXEC5 &&
						state != COMMAND_RW_DATA_EXEC6 &&
						state != COMMAND_RW_DATA_EXEC7)
              i_rpm_timer[i_current_drive][i] <= i_rpm_timer[i_current_drive][i] + 1'd1;
          end
        end
      end

      m_status[UPD765_MAIN_D0B] <= |seek_state[0];
      m_status[UPD765_MAIN_D1B] <= |seek_state[1];
      m_status[UPD765_MAIN_CB] <= state != COMMAND_IDLE;

      old_tc <= tc;
      if(~old_tc && tc && m_status[UPD765_MAIN_EXM]) begin // && ~sd_busy) begin		// TC signal jump straight to reading results
        state <= COMMAND_READ_RESULTS;  // TC caused drive to reset state
        phase <= PHASE_RESPONSE;
        m_status[UPD765_MAIN_EXM] <= 1'b0;
        case (last_state)
          COMMAND_SCAN_EQUAL,
				COMMAND_SCAN_LOW_OR_EQUAL,
				COMMAND_SCAN_HIGH_OR_EQUAL,
				COMMAND_READ_DATA,
				COMMAND_READ_DELETED_DATA,
				COMMAND_FORMAT_TRACK,
				COMMAND_WRITE_DATA:
          int_state[ds0] <= 1'b1;
          default: int_state[ds0] <= 1'b0;
        endcase
        //			int_state[ds0] <= 1'b1;
        i_substate <= 0;
      end else begin
        case (state)

        COMMAND_IDLE: begin
          //$display("COMANDO RECIBIDO: din = 0x%02x", din);
        
          // Imprimir bits decodificados
          //$display("Bits comando: MT=%b, SK=%b", din[7], din[5]);
        
          m_status[UPD765_MAIN_DIO] <= 0;
          m_status[UPD765_MAIN_RQM] <= !image_scan_state[0] & !image_scan_state[1];
          // reset tc
          //tc <= 1'b0;
          phase <= PHASE_COMMAND;
          if (~old_wr & wr & a0 & !image_scan_state[0] & !image_scan_state[1]) begin
            i_mt <= din[7];
            //i_mfm <= din[6];
            i_sk <= din[5];
        
            i_substate <= 0;
        
            casex (din[7:0])
              8'bXXX_00110: begin
                state <= COMMAND_READ_DATA;
                last_state <= COMMAND_READ_DATA;
              end
              8'bXXX_01100: begin
                state <= COMMAND_READ_DELETED_DATA;
                last_state <= COMMAND_READ_DELETED_DATA;
              end
              8'bXX0_00101: begin
                state <= COMMAND_WRITE_DATA;
                last_state <= COMMAND_WRITE_DATA;
              end
              8'bXX0_01001: begin
                state <= COMMAND_WRITE_DELETED_DATA;
                last_state <= COMMAND_WRITE_DELETED_DATA;
              end
              8'b0XX_00010: begin
                state <= COMMAND_READ_TRACK;
                last_state <= COMMAND_READ_TRACK;
              end
              8'b0X0_01010: begin
                state <= COMMAND_READ_ID;
                last_state <= COMMAND_READ_ID;
              end
              8'b0X0_01101: begin
                state <= COMMAND_FORMAT_TRACK;
                last_state <= COMMAND_FORMAT_TRACK;
              end
              8'b000_10001: begin
                state <= COMMAND_SCAN_EQUAL;
                last_state <= COMMAND_SCAN_EQUAL;
                i_scan_mode[ds0] <= 2'b01;
                $display("SCAN_EQUAL command detected, mode set to: %b", 2'b01);
              end
              8'b000_11001: begin
                state <= COMMAND_SCAN_LOW_OR_EQUAL;
                last_state <= COMMAND_SCAN_LOW_OR_EQUAL;
                i_scan_mode[ds0] <= 2'b10;
                $display("SCAN_LOW_OR_EQUAL command detected, mode set to: %b", 2'b10);
              end
              8'b000_11101: begin
                state <= COMMAND_SCAN_HIGH_OR_EQUAL;
                last_state <= COMMAND_SCAN_HIGH_OR_EQUAL;
                i_scan_mode[ds0] <= 2'b11;
                $display("SCAN_HIGH_OR_EQUAL command detected, mode set to: %b", 2'b11);
              end
              8'b000_00111: begin
                state <= COMMAND_RECALIBRATE;
                last_state <= COMMAND_RECALIBRATE;
              end
              8'b000_01000: begin
                state <= COMMAND_SENSE_INTERRUPT_STATUS;
                last_state <= COMMAND_SENSE_INTERRUPT_STATUS;
              end
              8'b000_00011: begin
                state <= COMMAND_SPECIFY;
                last_state <= COMMAND_SPECIFY;
              end
              8'b000_00100: begin
                state <= COMMAND_SENSE_DRIVE_STATUS;
                last_state <= COMMAND_SENSE_DRIVE_STATUS;
              end
              8'b000_01111: begin
                state <= COMMAND_SEEK;
                last_state <= COMMAND_SEEK;
              end
              default: begin
                state <= COMMAND_INVALID;
                last_state <= COMMAND_INVALID;
              end
            endcase
        
            $display("COMANDO RECIBIDO: din = 0x%02x (binario: %b)", din, din);
        
            // Descomponer los bits
            $display("Desglose de bits:");
            $display("Bit 7 (MT): %b", din[7]);
            $display("Bit 6: %b", din[6]);
            $display("Bit 5 (SK): %b", din[5]);
            $display("Bit 4-0: %b", din[4:0]);
        
          end else if (~old_rd & rd & a0) begin
            m_data <= 8'hff;
          end

        end

          COMMAND_SENSE_INTERRUPT_STATUS: begin
            m_status[UPD765_MAIN_DIO] <= 1;
            state <= COMMAND_SENSE_INTERRUPT_STATUS1;
          end

          COMMAND_SENSE_INTERRUPT_STATUS1:
          if (~old_rd & rd & a0) begin
            if (int_state[0]) begin
              m_data <= (ncn[0] == pcn[0] && image_ready[0]) ? 8'h20 : 8'he8;  //drive A: interrupt
              state  <= COMMAND_SENSE_INTERRUPT_STATUS2;
            end else if (int_state[1]) begin
              m_data <= (ncn[1] == pcn[1] && image_ready[1]) ? 8'h21 : 8'he9;  //drive B: interrupt
              state  <= COMMAND_SENSE_INTERRUPT_STATUS2;
            end else begin
              m_data <= 8'h80;
              state  <= COMMAND_IDLE;
            end
            ;
          end

          COMMAND_SENSE_INTERRUPT_STATUS2:
          if (~old_rd & rd & a0) begin
            // Devolver el PCN de la unidad que est?? reportando la interrupci??n
            m_data <= int_state[0] ? 
        ((image_density[0]==CF2 && density[0]==CF2DD) ? pcn[0] << 1 : pcn[0]) :  
        ((image_density[1]==CF2 && density[1]==CF2DD) ? pcn[1] << 1 : pcn[1]);

            //int_state <= '{ 0, 0 }; // Limpiar ambas interrupciones
            int_state[int_state[0]?0 : 1] <= 0;
            state <= COMMAND_IDLE;
          end

          COMMAND_SENSE_DRIVE_STATUS: begin
            int_state <= '{0, 0};
            if (~old_wr & wr & a0) begin
              state <= COMMAND_SENSE_DRIVE_STATUS_RD;
              m_status[UPD765_MAIN_DIO] <= 1;
              ds0 <= din[0];
              hds <= image_density[din[0]] ? din[2] : 1'b0;  // Was missing
            end
          end

          COMMAND_SENSE_DRIVE_STATUS_RD:
          if (~old_rd & rd & a0) begin
            m_data <= {
              1'b0,
              ready[ds0] & image_wp[ds0],  //write protected
              motor[ds0] & available[ds0],  //ready - needed for controller detection
              ready[ds0] & !pcn[ds0],  //track 0
              ready[ds0] & image_sides[ds0],  //two sides
              ready[ds0] & hds,  //head address
              1'b0,  //us1
              ds0
            };  //us0
            state <= COMMAND_IDLE;
          end

          COMMAND_SPECIFY: begin
            int_state <= '{0, 0};
            if (~old_wr & wr & a0) begin
              i_hut <= din[3:0];
              i_srt <= din[7:4];
              state <= COMMAND_SPECIFY_WR;
            end
          end

          COMMAND_SPECIFY_WR:
          if (~old_wr & wr & a0) begin
            i_hlt <= din[7:1];
            ndma_mode <= din[0];
            int_state[ds0] <= 1'b1;
            state <= COMMAND_IDLE;
          end

          COMMAND_RECALIBRATE: begin
            if (~old_wr & wr & a0) begin
              ds0 <= din[0];
              int_state[din[0]] <= 0;
              ncn[din[0]] <= 0;
              seek_state[din[0]] <= 1;
              state <= COMMAND_IDLE;
            end
          end

          COMMAND_SEEK: begin
            if (~old_wr & wr & a0) begin
              ds0 <= din[0];
              hds <= image_density[din[0]] ? din[2] : 1'b0;  // Was missing
              int_state[din[0]] <= 0;
              state <= COMMAND_SEEK_EXEC1;
            end
          end

          COMMAND_SEEK_EXEC1:
          if (~old_wr & wr & a0) begin
            // This next line is intentionally blocking
            if (image_density[ds0] == CF2 && density[ds0] == CF2DD) begin
              ncn[ds0] <= din >> 1;
              if ((motor[ds0] && ready[ds0] && image_ready[ds0] && (din >> 1)<image_tracks[ds0]) || !din) begin
                seek_state[ds0] <= 1;
              end else begin
                //Seek error
                int_state[ds0] <= 1;
              end
            end else begin
              ncn[ds0] <= din;
              if ((motor[ds0] && ready[ds0] && image_ready[ds0] && din<image_tracks[ds0]) || !din) begin
                seek_state[ds0] <= 1;
              end else begin
                //Seek error
                int_state[ds0] <= 1;
              end
            end
            state <= COMMAND_IDLE;
          end

          COMMAND_READ_ID: begin
            int_state <= '{0, 0};
            state <= COMMAND_READ_ID1;
          end

          COMMAND_READ_ID1:
          if (~old_wr & wr & a0) begin
            ds0 <= din[0];
            if (~motor[din[0]] | ~ready[din[0]] | ~image_ready[din[0]]) begin
              status[0] <= 8'h40;
              status[1] <= 8'b101;
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[din[0]] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else if (din[2] & ~image_sides[din[0]]) begin
              status[0] <= 8'h48;  //no side B
              status[1] <= 0;
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[din[0]] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else begin
              hds <= image_density[din[0]] ? din[2] : 1'b0;
              m_status[UPD765_MAIN_RQM] <= 0;
              i_command <= COMMAND_READ_ID2;
              state <= COMMAND_RELOAD_TRACKINFO;
              phase <= PHASE_EXECUTE;
            end
          end

          COMMAND_READ_ID2: begin
            image_track_offsets_addr <= {pcn[ds0], hds};
            buff_wait <= 1;
            state <= COMMAND_READ_ID_EXEC1;
          end

          COMMAND_READ_ID_EXEC1:
          if (~sd_busy & ~buff_wait) begin
            if (image_track_offsets_in) begin
              state <= COMMAND_READ_ID_WAIT_SECTOR;
            end else begin
              //empty track
              status[0] <= 8'h40;
              status[1] <= 8'b101;
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end
          end

          // Actually sets the offset to sector table in track (started with TRACKINFO)
          COMMAND_READ_ID_WAIT_SECTOR:
          if (~sd_busy & ~buff_wait & (!i_rpm_timer[ds0][hds])) begin
            sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
            // 18h = offset to list of sectors in sector table for current track
            buff_addr <= {
              image_track_offsets_in[0], 8'h18 + (i_current_sector_pos[ds0][hds] << 3)
            };  //get the current sectorInfo
            buff_wait <= 1;
            state <= COMMAND_READ_ID_EXEC2;
          end

          COMMAND_READ_ID_EXEC2:
          if (~buff_wait) begin
            if (buff_addr[2:0] == 8'h00) i_sector_c <= buff_data_in;
            else if (buff_addr[2:0] == 8'h01) i_sector_h <= buff_data_in;
            else if (buff_addr[2:0] == 8'h02) i_sector_r <= buff_data_in;
            else if (buff_addr[2:0] == 8'h03) begin
              i_sector_n <= buff_data_in;
              status[0] <= 0;
              status[1] <= 0;
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end
            buff_addr <= buff_addr + 1'd1;
            buff_wait <= 1;
          end

          COMMAND_READ_TRACK: begin
            int_state <= '{0, 0};
            i_command <= COMMAND_RW_DATA_EXEC;
            state <= COMMAND_SETUP;
            {i_rtrack, i_write, i_rw_deleted} <= 3'b100;
            i_scan_mode[ds0] <= 2'b00;  // Normal mode, no scan
          end

          COMMAND_WRITE_DATA: begin
            int_state <= '{0, 0};
            i_command <= COMMAND_RW_DATA_EXEC;
            state <= COMMAND_SETUP;
            {i_rtrack, i_write, i_rw_deleted} <= 3'b010;
            i_scan_mode[ds0] <= 2'b00;  // Normal mode, no scan
          end

          COMMAND_WRITE_DELETED_DATA: begin
            int_state <= '{0, 0};
            i_command <= COMMAND_RW_DATA_EXEC;
            state <= COMMAND_SETUP;
            {i_rtrack, i_write, i_rw_deleted} <= 3'b011;
            i_scan_mode[ds0] <= 2'b00;  // Normal mode, no scan
          end

          COMMAND_READ_DATA: begin
            int_state <= '{0, 0};
            i_command <= COMMAND_RW_DATA_EXEC;
            state <= COMMAND_SETUP;
            {i_rtrack, i_write, i_rw_deleted} <= 3'b000;
            i_scan_mode[ds0] <= 2'b00;  // Normal mode, no scan
          end

          COMMAND_READ_DELETED_DATA: begin
            int_state <= '{0, 0};
            i_command <= COMMAND_RW_DATA_EXEC;
            state <= COMMAND_SETUP;
            {i_rtrack, i_write, i_rw_deleted} <= 3'b001;
            i_scan_mode[ds0] <= 2'b00;  // Normal mode, no scan
          end

 
          // Corregir el estado COMMAND_RW_DATA_SCAN_COMPARE para manejar correctamente la comparación
          COMMAND_RW_DATA_SCAN_COMPARE: begin
            // El dato del sector ya está en m_data (configurado en EXEC6)
            // Ahora esperamos que el CPU escriba un dato para comparar

            // Chequear si el CPU está enviando un dato para comparar
            if (~old_wr & wr & a0) begin
              $display("SCAN_COMPARE: SectorData=0x%02x, CPUData=0x%02x, Mode=%b", m_data, din,
                       i_scan_mode[ds0]);

              // Hacer la comparación apropiada según el modo de SCAN
              case (i_scan_mode[ds0])
                2'b01: begin  // SCAN_EQUAL
                  // Solo hay coincidencia si los datos son exactamente iguales
                  if (m_data == din) begin
                    i_scan_match <= 1;
                    $display(
                        "SCAN_EQUAL: ¡Coincidencia encontrada! SectorData=0x%02x == CPUData=0x%02x",
                        m_data, din);
                  end else begin
                    $display("SCAN_EQUAL: Sin coincidencia. SectorData=0x%02x != CPUData=0x%02x",
                             m_data, din);
                  end
                end
                2'b10: begin  // SCAN_LOW_OR_EQUAL
                  // Hay coincidencia si el dato del sector es menor o igual al dato del CPU
                  if (m_data <= din) begin
                    i_scan_match <= 1;
                    $display(
                        "SCAN_LOW_OR_EQUAL: ¡Coincidencia encontrada! SectorData=0x%02x <= CPUData=0x%02x",
                        m_data, din);
                  end else begin
                    $display(
                        "SCAN_LOW_OR_EQUAL: Sin coincidencia. SectorData=0x%02x > CPUData=0x%02x",
                        m_data, din);
                  end
                end
                2'b11: begin  // SCAN_HIGH_OR_EQUAL
                  // Hay coincidencia si el dato del sector es mayor o igual al dato del CPU
                  if (m_data >= din) begin
                    i_scan_match <= 1;
                    $display(
                        "SCAN_HIGH_OR_EQUAL: ¡Coincidencia encontrada! SectorData=0x%02x >= CPUData=0x%02x",
                        m_data, din);
                  end else begin
                    $display(
                        "SCAN_HIGH_OR_EQUAL: Sin coincidencia. SectorData=0x%02x < CPUData=0x%02x",
                        m_data, din);
                  end
                end
              endcase

              // Decrementar contador de bytes a leer
              i_bytes_to_read <= i_bytes_to_read - 1'd1;
              i_timeout <= OVERRUN_TIMEOUT;

              // Si ya encontramos una coincidencia, terminamos la operación
              if (i_scan_match) begin
                $display("SCAN: Coincidencia encontrada, terminando operación SCAN");
                state <= COMMAND_RW_DATA_EXEC8;
                m_status[UPD765_MAIN_RQM] <= 0;  // Desactivar RQM mientras procesamos
              end  // Si no hay más bytes para leer en este sector, pasamos al siguiente paso
              else if (i_bytes_to_read <= 1) begin
                $display("SCAN: Fin de datos del sector alcanzado");
                state <= COMMAND_RW_DATA_EXEC8;
                m_status[UPD765_MAIN_RQM] <= 0;  // Desactivar RQM mientras procesamos
              end  // De lo contrario, continuamos con el siguiente byte
              else begin
                $display("SCAN: Continuando con el siguiente byte");
                state <= COMMAND_RW_DATA_EXEC6;
                m_status[UPD765_MAIN_RQM] <= 0;  // Necesario para correcta transición de estado
              end
            end  // Si el timeout expira, abortamos la operación
            else if (i_timeout <= 0) begin
              $display("SCAN: Timeout mientras esperaba datos de comparación del CPU");
              m_status[UPD765_MAIN_EXM] <= 0;
              status[0] <= 8'h40;  // Error - bit AT (Abnormal Termination)
              status[1] <= 8'h00;
              status[2] <= 8'h00;
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else begin
              // Decrementar el contador de timeout mientras esperamos
              i_timeout <= i_timeout - 1'd1;
            end
          end


          // Bloque completo COMMAND_SETUP
          COMMAND_SETUP:
          if (!old_wr & wr & a0) begin
            $display("COMMAND_SETUP: substate=%d, din=%h", i_substate, din);
            case (i_substate)
              0: begin
                ds0        <= din[0];  // device
                hds        <= image_density[din[0]] ? din[2] : 1'b0;  // head polarity
                i_substate <= 1;
              end
              1: begin
                i_c        <= din;  // track
                i_substate <= 2;
              end
              2: begin
                i_h        <= image_density[ds0] ? din : 8'b0;  // head
                i_substate <= 3;
              end
              3: begin
                i_r        <= din;  // sector
                i_substate <= 4;
              end
              4: begin
                i_n        <= din;  // sector len (1 = 256, 2 = 512)
                i_substate <= 5;
              end
              5: begin
                i_eot      <= din;  // last sector in track
                i_substate <= 6;
              end
              6: begin
                //i_gpl <= din;     // gap len (seems to be ignored)
                i_substate <= 7;
              end
              7: begin
                // Para comandos SCAN, usar el último parámetro como STP en lugar de DTL
                if (i_scan_mode[ds0] != 2'b00) begin
                  i_stp <= din & 2'b11;  // Los 2 bits inferiores son el valor STP
                  $display("SCAN command, using STP=%d from input=%h", din & 2'b11, din);
                end else begin
                  i_dtl <= din;  // Para comandos normales, este es DTL
                end

                // Siempre pasar a validación después del último parámetro
                i_substate <= 0;
                state <= COMMAND_SETUP_VALIDATION;
              end
            endcase
          end

          // Bloque completo COMMAND_RW_DATA_EXEC8
          COMMAND_RW_DATA_EXEC3:
          if (~sd_busy & ~buff_wait) begin
            $display(
                "COMMAND_RW_DATA_EXEC3: buff_addr=%h, i_current_sector=%d, i_total_sectors=%d, image_ready=%d",
                buff_addr[7:0], i_current_sector, i_total_sectors, image_ready[ds0]);
                $display("EXEC3: Checking sector %d/%d, current sector info: C=%d, H=%d, R=%d, N=%d",
                i_current_sector, i_total_sectors, i_sector_c, i_sector_h, i_sector_r, i_sector_n);
            if (buff_addr[7:0] == 8'h14) begin
              if (!image_edsk[ds0]) i_sector_size <= 8'h80 << buff_data_in[2:0];
              buff_addr[7:0] <= 8'h18;  //sector info list
              buff_wait <= 1;
              $display("Setting sector size and moving to sector info list");
            end else if (i_current_sector > i_total_sectors) begin
              $display("ERROR: Sector not found or end of track - current=%d, total=%d",
                       i_current_sector, i_total_sectors);
              m_status[UPD765_MAIN_EXM] <= 0;
              //sector not found or end of track
              status[0] <= i_rtrack ? 8'h00 : 8'h40;
              status[1] <= i_rtrack ? 8'h00 : 8'h04;
              status[2] <= i_rtrack | ~i_bc ? 8'h00 : (i_sector_c == 8'hff ? 8'h02 : 8'h10); //bad/wrong cylinder
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else begin
              //process sector info list
              case (buff_addr[2:0])
                0: begin
                  i_sector_c <= buff_data_in;
                  $display("Sector C=%h", buff_data_in);
                end
                1: begin
                  i_sector_h <= buff_data_in;
                  $display("Sector H=%h", buff_data_in);
                end
                2: begin
                  i_sector_r <= buff_data_in;
                  $display("Sector R=%h", buff_data_in);
                end
                3: begin
                  i_sector_n <= buff_data_in;
                  $display("Sector N=%h", buff_data_in);
                end
                4: begin
                  i_sector_st1 <= buff_data_in;
                  $display("Sector ST1=%h", buff_data_in);
                end
                5: begin
                  i_sector_st2 <= buff_data_in;
                  $display("Sector ST2=%h", buff_data_in);
                end
                6: begin
                  if (image_edsk[ds0]) i_sector_size[7:0] <= buff_data_in;
                  $display("Sector size low=%h", buff_data_in);
                end
                7: begin
                  // start scanning of the sector IDs from the sector at the current head position
                  if (image_edsk[ds0]) i_sector_size[15:8] <= buff_data_in;
                  $display("Sector size high=%h, moving to EXEC4", buff_data_in);
                  state <= COMMAND_RW_DATA_EXEC4;
                end
              endcase
              buff_addr <= buff_addr + 1'd1;
              buff_wait <= 1;
            end
          end
          COMMAND_RW_DATA_EXEC4:
if ((i_rtrack && i_current_sector == i_r) ||
    (~i_rtrack && i_sector_c == i_c && i_sector_r == i_r && i_sector_h == i_h && (i_sector_n == i_n || !i_n))) begin
  $display("EXEC4: Looking for C=%d, H=%d, R=%d, N=%d", i_c, i_h, i_r, i_n);
  $display("EXEC4: Found C=%d, H=%d, R=%d, N=%d", i_sector_c, i_sector_h, i_sector_r, i_sector_n);
  //sector found in the sector info list
  if (i_sk & ~i_rtrack & (i_rw_deleted ^ i_sector_st2[6])) begin
    $display("EXEC4 to EXEC8:");
    state <= COMMAND_RW_DATA_EXEC8;
  end else begin
    i_bytes_to_read <= i_n ? (8'h80 << (i_n[3] ? 4'h8 : i_n[2:0])) : i_dtl;
    i_timeout <= OVERRUN_TIMEOUT;
    i_weak_sector <= 0;
    state <= COMMAND_RW_DATA_WAIT_SECTOR;
  end
end else begin
  //try the next sector in the sectorinfo list
  $display("EXEC4 Next Sector:");
  if (i_sector_c == i_c) i_bc <= 0;
  i_current_sector <= i_current_sector + 1'd1;
  i_seek_pos <= i_seek_pos + i_sector_size;
  state <= COMMAND_RW_DATA_EXEC3;
end

COMMAND_RW_DATA_EXEC6:
if (~sd_busy & ~buff_wait) begin
  $display("RW_DATA_EXEC6: i_bytes_to_read=%d, m_status=%h", i_bytes_to_read, m_status);

  if (!i_bytes_to_read) begin
    //end of the current sector in buffer, so write it to SD card
    if (i_write && buff_addr && i_seek_pos < image_size[ds0]) begin
      sd_lba <= i_seek_pos[31:9];
      sd_wr[ds0] <= 1;
      sd_busy <= 1;
    end
    state <= COMMAND_RW_DATA_EXEC8;
  end else if (~m_status[UPD765_MAIN_RQM]) begin
    m_status[UPD765_MAIN_RQM] <= 1;
    if (ndma_mode) int_state[ds0] <= 1'b1;
  end else if (~i_write & ~old_rd & rd & a0) begin
    if (&buff_addr) begin
      //sector continues on the next LBA
      state <= COMMAND_RW_DATA_EXEC5;
    end

    // Operaciones de lectura normal
    m_data <= buff_data_in;
    m_status[UPD765_MAIN_RQM] <= 0;
    if (i_sector_size) begin
      i_sector_size <= i_sector_size - 1'd1;
      buff_addr <= buff_addr + 1'd1;
      buff_wait <= 1;
      i_seek_pos <= i_seek_pos + 1'd1;
    end
    i_bytes_to_read <= i_bytes_to_read - 1'd1;
    i_timeout <= OVERRUN_TIMEOUT;
    if (ndma_mode) int_state[ds0] <= 1'b0;
  end else if (i_write & ~old_wr & wr & a0) begin
    buff_wr <= 1;
    buff_data_out <= din;
    i_timeout <= OVERRUN_TIMEOUT;
    m_status[UPD765_MAIN_RQM] <= 0;
    state <= COMMAND_RW_DATA_EXEC7;
    if (ndma_mode) int_state[ds0] <= 1'b0;
  end else begin
    i_timeout <= i_timeout - 1'd1;
  end
end

COMMAND_RW_DATA_EXEC8:
if (~sd_busy) begin
  $display("RW_DATA_EXEC8: Normal Read/Write command");

  if (~i_rtrack & ~(i_sk & (i_rw_deleted ^ i_sector_st2[6])) &
      ((i_sector_st1[5] & i_sector_st2[5]) | (i_rw_deleted ^ i_sector_st2[6]))) begin
    // Marca borrada o error CRC
    m_status[UPD765_MAIN_EXM] <= 0;
    status[0] <= 8'h40;
    status[1] <= i_sector_st1;
    status[2] <= i_sector_st2 | (i_rw_deleted ? 8'h40 : 8'h0);
    state <= COMMAND_READ_RESULTS;
    int_state[ds0] <= 1'b1;
    phase <= PHASE_RESPONSE;
  end else if ((i_rtrack ? i_current_sector : i_sector_r) == i_eot) begin
    // Fin de cilindro
    m_status[UPD765_MAIN_EXM] <= 0;

    // Manejo para comandos normales
    status[0] <= i_rtrack ? 8'h00 : 8'h00;
    status[1] <= 8'h00;
    status[2] <= (i_rw_deleted ^ i_sector_st2[6]) ? 8'h40 : 8'h0;

    state <= COMMAND_READ_RESULTS;
    int_state[ds0] <= 1'b1;
    phase <= PHASE_RESPONSE;
  end else begin
    // Leer el siguiente sector (transferencia multi-sector)
    if (i_mt & image_sides[ds0]) begin
      hds <= ~hds;
      i_h <= ~i_h;
      image_track_offsets_addr <= {pcn[ds0], ~hds};
      buff_wait <= 1;
    end

    // Incremento normal para comandos estándar
    if (~i_mt | hds | ~image_sides[ds0]) begin
      i_r <= i_r + 1'd1;
    end

    state <= COMMAND_RW_DATA_EXEC2;
  end
end

          // Fix the COMMAND_READ_RESULTS state to correctly return the status registers
          COMMAND_READ_RESULTS: begin
            phase <= PHASE_RESPONSE;
            m_status[UPD765_MAIN_DIO] <= 1;
            if (~sd_busy & ~buff_wait) begin
              m_status[UPD765_MAIN_RQM] <= 1;
              if (~old_rd & rd & a0) begin
                case (i_substate)
                  0: begin
                    m_data <= {status[0][7:3], hds, 1'b0, ds0};  // status[0][7:3]
                    i_substate <= 1;
                    int_state[ds0] <= 1'b0;
                    $display("READ_RESULTS: ST0=0x%02x (bits 7-3=%b, hds=%b, ds0=%b)", status[0],
                             status[0][7:3], hds, ds0);
                  end
                  1: begin
                    m_data <= status[1];
                    i_substate <= 2;
                    $display("READ_RESULTS: ST1=0x%02x", status[1]);
                  end
                  2: begin
                    m_data <= status[2];
                    i_substate <= 3;
                    $display("READ_RESULTS: ST2=0x%02x", status[2]);
                  end
                  3: begin
                    m_data <= i_sector_c;
                    i_substate <= 4;
                    $display("READ_RESULTS: C=0x%02x", i_sector_c);
                  end
                  4: begin
                    m_data <= i_sector_h;
                    i_substate <= 5;
                    $display("READ_RESULTS: H=0x%02x", i_sector_h);
                  end
                  5: begin
                    m_data <= i_sector_r;
                    i_substate <= 6;
                    $display("READ_RESULTS: R=0x%02x", i_sector_r);
                  end
                  6: begin
                    m_data <= i_sector_n;
                    state  <= COMMAND_IDLE;
                    $display("READ_RESULTS: N=0x%02x", i_sector_n);
                  end
                  7: ;  //not happen
                endcase
              end
            end else m_status[UPD765_MAIN_RQM] <= 0;
          end

          COMMAND_SCAN_EQUAL: begin
            int_state <= '{0, 0};
            i_scan_mode[ds0] <= 2'b01;  // SCAN_EQUAL mode
            i_scan_match <= 0;     // Reset match flag
            state <= COMMAND_SETUP; // Reutilizar configuración inicial
          end
          
          COMMAND_SCAN_LOW_OR_EQUAL: begin
            int_state <= '{0, 0};
            i_scan_mode[ds0] <= 2'b10;  // SCAN_LOW_OR_EQUAL mode
            i_scan_match <= 0;     // Reset match flag
            state <= COMMAND_SETUP; // Reutilizar configuración inicial
          end
          
          COMMAND_SCAN_HIGH_OR_EQUAL: begin
            int_state <= '{0, 0};
            i_scan_mode[ds0] <= 2'b11;  // SCAN_HIGH_OR_EQUAL mode
            i_scan_match <= 0;     // Reset match flag
            state <= COMMAND_SETUP; // Reutilizar configuración inicial
          end
          // Ensure SETUP_VALIDATION also resets i_scan_match for each new command

          // Fix the initialization of SCAN mode variables
          COMMAND_RESET: begin
            old_tc <= 1'b0;
            m_status <= 8'h80;
            state <= COMMAND_IDLE;
            last_state <= COMMAND_IDLE;
            phase <= PHASE_COMMAND;
            status[0] <= 0;
            status[1] <= 0;
            status[2] <= 0;
            status[3] <= 0;
            ncn <= '{0, 0};
            pcn <= '{0, 0};
            int_state <= '{0, 0};
            seek_state <= '{0, 0};
            image_trackinfo_dirty <= '{1, 1};
            {ack, sd_busy} <= 0;
            sd_rd <= 0;
            sd_wr <= 0;
            sd_busy <= 0;
            image_track_offsets_wr <= 0;
            //restart "mounting" of image(s)
            if (image_scan_state[0]) image_scan_state[0] <= 1;
            if (image_scan_state[1]) image_scan_state[1] <= 1;
            i_scan_lock  <= 0;
            i_srt        <= 4;
            ndma_mode    <= 1'b1;
            i_scan_mode[ds0]  <= 2'b00;  // Initialize scan mode to normal (not SCAN)
            i_scan_match <= 0;  // Initialize scan match flag
            i_stp        <= 2'b01;  // Default STP value
          end


          // Add logs to COMMAND_RW_DATA_EXEC1
          COMMAND_RW_DATA_EXEC1: begin
            $display("COMMAND_RW_DATA_EXEC1: scan_mode=%b", i_scan_mode[ds0]);
            m_status[UPD765_MAIN_DIO] <= ~i_write;
            if (i_rtrack) i_r <= 1;
            i_bc <= 1;
            i_scan_match <= 0;  // Initialize match for SCAN commands

            // Read from the track stored at the last seek
            // even if different one is given in the command
            image_track_offsets_addr <= {pcn[ds0], hds};
            buff_wait <= 1;
            state <= COMMAND_RW_DATA_EXEC2;
            $display("Moving to COMMAND_RW_DATA_EXEC2");
          end

          // Add logs to COMMAND_RW_DATA_EXEC2
          COMMAND_RW_DATA_EXEC2: begin
            $display("COMMAND_RW_DATA_EXEC2: sd_busy=%b, buff_wait=%b", sd_busy, buff_wait);

            if (~sd_busy & ~buff_wait) begin
              $display("Setting up track info and sector read");
              i_current_sector <= 1'd1;
              sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
              i_seek_pos <= {image_track_offsets_in + 1'd1, 8'd0};  //TrackInfo+256bytes
              buff_addr <= {image_track_offsets_in[0], 8'h14};  //sector size
              buff_wait <= 1;
              state <= COMMAND_RW_DATA_EXEC3;
            end
          end

 
          //wait for the sector needed for positioning at the head - delay only
          COMMAND_RW_DATA_WAIT_SECTOR:
          if ((i_current_sector_pos[ds0][hds] == i_current_sector - 1'd1) && !i_rpm_timer[ds0][hds]) begin
            m_status[UPD765_MAIN_EXM] <= 1;
            state <= COMMAND_RW_DATA_EXEC_WEAK;
          end

          // Copy protection, PCW skips to RW_DATA_EXEC5 (not true for EDSK)
          COMMAND_RW_DATA_EXEC_WEAK:
          if (image_edsk[ds0] && (i_sector_size == {i_bytes_to_read, 1'b0} ||  // 2 weak sectors
              (i_sector_size == ({i_bytes_to_read, 1'b0} + i_bytes_to_read)) ||  // 3 weak sectors
              (i_sector_size == {i_bytes_to_read, 2'b00}))) begin  // 4 weak sectors
            //if sector data == 2,3,4x sector size, then handle multiple version of the same sector (weak sectors)
            //otherwise extra data is considered as GAP data
            if (i_weak_sector != next_weak_sector[ds0]) begin
              i_seek_pos <= i_seek_pos + i_bytes_to_read;
              i_sector_size <= i_sector_size - i_bytes_to_read;
              i_weak_sector <= i_weak_sector + 1'd1;
            end else begin
              next_weak_sector[ds0] <= next_weak_sector[ds0] + 1'd1;
              state <= COMMAND_RW_DATA_EXEC5;
            end
          end else begin
            if (SPECCY_SPEEDLOCK_HACK & 
						i_current_sector == 2 & !pcn[ds0] & ~hds & i_sector_st1[5] & i_sector_st2[5])
              next_weak_sector[ds0] <= next_weak_sector[ds0] + 1'd1;
            else next_weak_sector[ds0] <= 0;
            //					if (i_bytes_to_read > i_sector_size) i_bytes_to_read <= i_sector_size;
            state <= COMMAND_RW_DATA_EXEC5;
          end

          //Read the LBA for the sector into the RAM
          COMMAND_RW_DATA_EXEC5:
          if (~sd_busy & ~buff_wait) begin
            sd_buff_type <= UPD765_SD_BUFF_SECTOR;
            sd_rd[ds0] <= 1;
            sd_lba <= i_seek_pos[31:9];
            sd_busy <= 1;
            buff_addr <= i_seek_pos[8:0];
            buff_wait <= 1;
            state <= COMMAND_RW_DATA_EXEC6;
          end

          COMMAND_RW_DATA_EXEC7: begin
            buff_wr <= 0;
            if (i_sector_size) begin
              i_sector_size <= i_sector_size - 1'd1;
              buff_addr <= buff_addr + 1'd1;
              buff_wait <= 1;
              i_seek_pos <= i_seek_pos + 1'd1;
            end
            i_bytes_to_read <= i_bytes_to_read - 1'd1;
            if (&buff_addr) begin
              //sector continues on the next LBA
              //so write out the current before reading the next
              if (i_seek_pos < image_size[ds0]) begin
                sd_lba <= i_seek_pos[31:9];
                sd_wr[ds0] <= 1;
                sd_busy <= 1;
              end
              state <= COMMAND_RW_DATA_EXEC5;
            end else begin
              state <= COMMAND_RW_DATA_EXEC6;
            end
          end


          COMMAND_SCAN_EXEC1: begin
            $display("COMMAND_SCAN_EXEC1: Starting SCAN operation, mode=%b", i_scan_mode[ds0]);
            $display("SCAN parameters: C=%d, H=%d, R=%d, N=%d, EOT=%d", i_c, i_h, i_r, i_n, i_eot);
            
            m_status[UPD765_MAIN_DIO] <= 0;
            i_bc <= 1;
            i_scan_match <= 0;
            
            // Forzar la recarga de la información de pista
            image_trackinfo_dirty[ds0] <= 1;
            
            // Y asegurarnos de que usamos el PCN correcto
            if (pcn[ds0] != i_c) begin
              $display("SCAN: Forcing head movement from track %d to %d", pcn[ds0], i_c);
              pcn[ds0] <= i_c;
            end
            
            m_status[UPD765_MAIN_RQM] <= 0;
            i_command <= COMMAND_SCAN_EXEC2;
            state <= COMMAND_RELOAD_TRACKINFO;
          end
          
          COMMAND_SCAN_EXEC2: begin
            $display("COMMAND_SCAN_EXEC2: Loading track info");
            if (~sd_busy & ~buff_wait) begin
              i_current_sector <= 1'd1;
              sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
              i_seek_pos <= {image_track_offsets_in + 1'd1, 8'd0}; //TrackInfo+256bytes
              buff_addr <= {image_track_offsets_in[0], 8'h14}; //sector size
              buff_wait <= 1;
              state <= COMMAND_SCAN_EXEC3;
            end
          end
          
          COMMAND_SCAN_EXEC3: begin
            $display("COMMAND_SCAN_EXEC3: Processing sector info, buff_addr=%h, current=%d, total=%d", 
                     buff_addr, i_current_sector, i_total_sectors);
            
            if (~sd_busy & ~buff_wait) begin
              // Añadir más logging para diagnosticar
              $display("SCAN_EXEC3: buff_data_in = %h at address %h", buff_data_in, buff_addr);
              
              if (buff_addr[7:0] == 8'h14) begin
                if (!image_edsk[ds0]) begin
                  i_sector_size <= 8'h80 << buff_data_in[2:0];
                  $display("Setting sector size from track info: %d", 8'h80 << buff_data_in[2:0]);
                end
                buff_addr[7:0] <= 8'h18; // Sector info list
                buff_wait <= 1;
              end else if (i_current_sector > i_total_sectors) begin
                // Sector no encontrado o fin de pista
                $display("Sector not found: current=%d, total=%d", i_current_sector, i_total_sectors);
                m_status[UPD765_MAIN_EXM] <= 0;
                status[0] <= 8'h40; // Abnormal termination
                status[1] <= 8'h04; // Sector not found
                status[2] <= 0;
                state <= COMMAND_READ_RESULTS;
                int_state[ds0] <= 1'b1;
                phase <= PHASE_RESPONSE;
              end else begin
                // Procesar la lista de info de sectores
                case (buff_addr[2:0])
                  0: begin
                    i_sector_c <= buff_data_in;
                    $display("Sector[%d] C=%h", i_current_sector, buff_data_in);
                  end
                  1: begin
                    i_sector_h <= buff_data_in;
                    $display("Sector[%d] H=%h", i_current_sector, buff_data_in);
                  end
                  2: begin
                    i_sector_r <= buff_data_in;
                    $display("Sector[%d] R=%h", i_current_sector, buff_data_in);
                  end
                  3: begin
                    i_sector_n <= buff_data_in;
                    $display("Sector[%d] N=%h", i_current_sector, buff_data_in);
                  end
                  4: begin
                    i_sector_st1 <= buff_data_in;
                    $display("Sector[%d] ST1=%h", i_current_sector, buff_data_in);
                  end
                  5: begin
                    i_sector_st2 <= buff_data_in;
                    $display("Sector[%d] ST2=%h", i_current_sector, buff_data_in);
                  end
                  6: begin 
                    if (image_edsk[ds0]) begin
                      i_sector_size[7:0] <= buff_data_in;
                      $display("Sector[%d] size low=%h", i_current_sector, buff_data_in);
                    end
                  end
                  7: begin
                    if (image_edsk[ds0]) begin
                      i_sector_size[15:8] <= buff_data_in;
                      $display("Sector[%d] size high=%h", i_current_sector, buff_data_in);
                    end
                    $display("Sector[%d] info complete: C=%d, H=%d, R=%d, N=%d", 
                            i_current_sector, i_sector_c, i_sector_h, i_sector_r, i_sector_n);
                    
                    // Verificar la dirección del buffer para descartar problemas
                    $display("Current buffer address: %h, seek_pos: %h", buff_addr, i_seek_pos);
                    
                    state <= COMMAND_SCAN_EXEC4;
                  end
                endcase
                buff_addr <= buff_addr + 1'd1;
                buff_wait <= 1;
              end
            end
          end
          
          COMMAND_SCAN_EXEC4: begin
            $display("COMMAND_SCAN_EXEC4: Checking sector match");
            $display("Looking for: C=%d, H=%d, R=%d, N=%d", i_c, i_h, i_r, i_n);
            $display("Found: C=%d, H=%d, R=%d, N=%d", i_sector_c, i_sector_h, i_sector_r, i_sector_n);
            
            // Verificar si encontramos el sector correcto
            if (i_sector_c == i_c && i_sector_r == i_r && i_sector_h == i_h && (i_sector_n == i_n || !i_n)) begin
              // Sector encontrado
              $display("Sector match found!");
              i_bytes_to_read <= i_n ? (8'h80 << (i_n[3] ? 4'h8 : i_n[2:0])) : i_dtl;
              i_timeout <= OVERRUN_TIMEOUT;
              state <= COMMAND_SCAN_READ_SECTOR;
            end else begin
              // Probar con el siguiente sector
              $display("Sector mismatch, trying next sector");
              if (i_sector_c == i_c) i_bc <= 0;
              i_current_sector <= i_current_sector + 1'd1;
              i_seek_pos <= i_seek_pos + i_sector_size;
              state <= COMMAND_SCAN_EXEC3;
            end
          end
          
          COMMAND_SCAN_READ_SECTOR: begin
            $display("COMMAND_SCAN_READ_SECTOR: Reading sector data");
            if (~sd_busy & ~buff_wait) begin
              // Leer el sector del disco
              sd_buff_type <= UPD765_SD_BUFF_SECTOR;
              sd_rd[ds0] <= 1;
              sd_lba <= i_seek_pos[31:9];
              sd_busy <= 1;
              buff_addr <= i_seek_pos[8:0];
              buff_wait <= 1;
              state <= COMMAND_SCAN_COMPARE;
            end
          end
          
          COMMAND_SCAN_COMPARE: begin
            if (~sd_busy & ~buff_wait) begin
              // Establecer flags para indicar que necesitamos datos
              m_status[UPD765_MAIN_RQM] <= 1;
              m_status[UPD765_MAIN_DIO] <= 0;
              
              // Generar una interrupción si estamos en modo no-DMA
              if (ndma_mode) int_state[ds0] <= 1'b1;
              
              if (~old_wr & wr & a0) begin
                // El CPU ha enviado datos, procesarlos normalmente
                $display("SCAN comparison: sector data=0x%02X, CPU data=0x%02X, mode=%b", 
                         buff_data_in, din, i_scan_mode[ds0]);
                
                // Comparación según el modo
                case (i_scan_mode[ds0])
                  2'b01: begin // SCAN_EQUAL
                    if (buff_data_in == din) begin
                      i_scan_match <= 1;
                      $display("SCAN_EQUAL match found!");
                    end
                  end
                  2'b10: begin // SCAN_LOW_OR_EQUAL
                    if (buff_data_in <= din) begin
                      i_scan_match <= 1;
                      $display("SCAN_LOW_OR_EQUAL match found!");
                    end
                  end
                  2'b11: begin // SCAN_HIGH_OR_EQUAL
                    if (buff_data_in >= din) begin
                      i_scan_match <= 1;
                      $display("SCAN_HIGH_OR_EQUAL match found!");
                    end
                  end
                endcase
                
                // Avanzar al siguiente byte
                i_bytes_to_read <= i_bytes_to_read - 1'd1;
                i_sector_size <= i_sector_size - 1'd1;
                buff_addr <= buff_addr + 1'd1;
                buff_wait <= 1;
                i_seek_pos <= i_seek_pos + 1'd1;
                i_timeout <= OVERRUN_TIMEOUT;
                
                // Desactivar interrupción ya que hemos procesado el byte
                if (ndma_mode) int_state[ds0] <= 1'b0;
                
                // Comprobar si hemos terminado
                if (i_scan_match || i_bytes_to_read <= 1) begin
                  state <= COMMAND_SCAN_NEXT;
                end else if (&buff_addr) begin
                  state <= COMMAND_SCAN_READ_SECTOR;
                end else begin
                  state <= COMMAND_SCAN_COMPARE;
                end
              end else if (i_timeout == 0) begin
                // Timeout: el CPU no envió dato para comparar
                $display("Timeout waiting for CPU data");
                m_status[UPD765_MAIN_EXM] <= 0;
                status[0] <= 8'h40; // Abnormal termination
                status[1] <= 0;
                status[2] <= 0;
                state <= COMMAND_READ_RESULTS;
                int_state[ds0] <= 1'b1;
                phase <= PHASE_RESPONSE;
              end else begin
                i_timeout <= i_timeout - 1'd1;
              end
            end
          end

          COMMAND_SCAN_NEXT: begin
            $display("COMMAND_SCAN_NEXT: Finalizing scan operation");
            // Decidir qué hacer después de comparar un sector
            if (i_scan_match || i_r == i_eot) begin
              // Terminar si hay coincidencia o llegamos al último sector
              $display("Ending scan: match=%b, r=%d, eot=%d", i_scan_match, i_r, i_eot);
              m_status[UPD765_MAIN_EXM] <= 0;
              status[0] <= 8'h00;
              status[1] <= 8'h00;
              
              // Configurar ST2 con los bits de resultado de SCAN apropiados
              if (i_scan_match) begin
                case (i_scan_mode[ds0])
                  2'b01: begin
                    status[2] <= 8'h10; // SCAN_EQUAL satisfecho (bits 2-3 = 11)
                    $display("Setting ST2=0x10 for SCAN_EQUAL match");
                  end
                  2'b10: begin 
                    status[2] <= 8'h08; // SCAN_LOW_OR_EQUAL satisfecho (bits 2-3 = 10)
                    $display("Setting ST2=0x08 for SCAN_LOW_OR_EQUAL match");
                  end
                  2'b11: begin
                    status[2] <= 8'h08; // SCAN_HIGH_OR_EQUAL satisfecho (bits 2-3 = 10)
                    $display("Setting ST2=0x08 for SCAN_HIGH_OR_EQUAL match");
                  end
                endcase
              end else begin
                status[2] <= 8'h00; // No se encontró coincidencia
                $display("Setting ST2=0x00 for no match");
              end
              
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else begin
              // Continuar con el siguiente sector según STP
              $display("Moving to next sector with STP=%d", i_stp);
              case (i_stp)
                2'b00, 2'b01: begin
                  i_r <= i_r + 1'd1; // STP=1: siguiente sector
                  $display("STP=1, next sector: %d", i_r + 1'd1);
                end
                2'b10: begin 
                  i_r <= i_r + 2'd2; // STP=2: saltar un sector
                  $display("STP=2, next sector: %d", i_r + 2'd2);
                end
                2'b11: begin
                  i_r <= i_r + 2'd3; // STP=3: saltar dos sectores
                  $display("STP=3, next sector: %d", i_r + 2'd3);
                end
              endcase
              
              // Reiniciar para la siguiente búsqueda
              i_scan_match <= 0;
              state <= COMMAND_SCAN_EXEC2;
            end
          end

          COMMAND_FORMAT_TRACK: begin
            int_state <= '{0, 0};
            if (~old_wr & wr & a0) begin
              ds0   <= din[0];
              state <= COMMAND_FORMAT_TRACK1;
            end
          end

          COMMAND_FORMAT_TRACK1:  //doesn't modify the media
          if (~old_wr & wr & a0) begin
            i_n   <= din;
            state <= COMMAND_FORMAT_TRACK2;
          end

          COMMAND_FORMAT_TRACK2:
          if (~old_wr & wr & a0) begin
            i_sc  <= din;
            state <= COMMAND_FORMAT_TRACK3;
          end

          COMMAND_FORMAT_TRACK3:
          if (~old_wr & wr & a0) begin
            //i_gpl <= din;
            state <= COMMAND_FORMAT_TRACK4;
          end

          COMMAND_FORMAT_TRACK4:
          if (~old_wr & wr & a0) begin
            //i_d <= din;
            m_status[UPD765_MAIN_EXM] <= 1;
            state <= COMMAND_FORMAT_TRACK5;
          end

          COMMAND_FORMAT_TRACK5: begin
            phase <= PHASE_EXECUTE;
            if (!i_sc) begin
              m_status[UPD765_MAIN_EXM] <= 0;
              status[0] <= 0;
              status[1] <= 0;
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else if (~old_wr & wr & a0) begin
              i_c   <= din;
              state <= COMMAND_FORMAT_TRACK6;
            end
          end

          COMMAND_FORMAT_TRACK6:
          if (~old_wr & wr & a0) begin
            i_h   <= image_density[ds0] ? din : 8'b0;
            state <= COMMAND_FORMAT_TRACK7;
          end

          COMMAND_FORMAT_TRACK7:
          if (~old_wr & wr & a0) begin
            i_r   <= din;
            state <= COMMAND_FORMAT_TRACK8;
          end

          COMMAND_FORMAT_TRACK8:
          if (~old_wr & wr & a0) begin
            i_n   <= din;
            i_sc  <= i_sc - 1'd1;
            i_r   <= i_r + 1'd1;
            state <= COMMAND_FORMAT_TRACK5;
          end

          // Fix for the COMMAND_SCAN_SETUP state
          COMMAND_SCAN_SETUP:
          if (!old_wr & wr & a0) begin
            i_stp <= din & 8'h03;
            $display("SCAN-SETUP: STP value = %d", din & 8'h03);

            // Move to the execute phase with proper status checks
            if (~motor[ds0] | ~ready[ds0] | ~image_ready[ds0]) begin
              status[0] <= 8'h40;
              status[1] <= 8'b101;
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else if (hds & ~image_sides[ds0]) begin
              hds <= 0;
              status[0] <= 8'h48;
              status[1] <= 0;
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else begin
              phase <= PHASE_EXECUTE;
              state <= i_command;
            end
          end



          COMMAND_INVALID: begin
            int_state <= '{0, 0};
            m_status[UPD765_MAIN_DIO] <= 1;
            //					m_status[UPD765_MAIN_RQM] <= 1;
            status[0] <= 8'h80;
            state <= COMMAND_INVALID1;
          end

          COMMAND_INVALID1:
          if (~old_rd & rd & a0) begin
            state  <= COMMAND_IDLE;
            m_data <= status[0];
            //					int_state[ds0] <= 1'b1;
          end


          COMMAND_SETUP_VALIDATION: begin
            $display("COMMAND_SETUP_VALIDATION: scan_mode=%b, i_command=%d", i_scan_mode[ds0],
                     i_command);
            $display("Disk conditions: motor=%b, ready=%b, image_ready=%b", motor[ds0], ready[ds0],
                     image_ready[ds0]);
          
            if (~motor[ds0] | ~ready[ds0]) begin
              $display("ERROR: Disk not ready - motor=%b, ready=%b, image_ready=%b", motor[ds0],
                       ready[ds0], image_ready[ds0]);
              status[0] <= 8'h40;
              status[1] <= 8'b101;
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else if (hds & ~image_sides[ds0]) begin
              $display("ERROR: No side B available - hds=%b, image_sides=%b", hds,
                       image_sides[ds0]);
              hds <= 0;
              status[0] <= 8'h48;  //no side B
              status[1] <= 0;
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else begin
              phase <= PHASE_EXECUTE;
              
              // Redirigir a los estados específicos de SCAN si es necesario
              if (i_scan_mode[ds0] != 2'b00) begin
                i_scan_match <= 0;  // Reset match flag
                $display("Dirigiendo a los estados específicos de SCAN: scan_mode=%b", i_scan_mode[ds0]);
                state <= COMMAND_SCAN_EXEC1;  // Estado específico para SCAN
              end else begin
                $display("Procediendo con el comando normal: i_command=%d", i_command);
                state <= i_command;
              end
            end
          end

 
         
          

          // Añadir logs al estado COMMAND_RW_DATA_EXEC
          COMMAND_RW_DATA_EXEC: begin
            $display("COMMAND_RW_DATA_EXEC: scan_mode=%b, write=%b, wp=%b", i_scan_mode[ds0], i_write,
                     image_wp[ds0]);
            if (i_scan_mode[ds0] != 2'b00) begin
              i_write <= 1'b0;
            end

            if (i_write & image_wp[ds0]) begin
              $display("ERROR: Disk is write protected");
              status[0] <= 8'h40;
              status[1] <= 8'h02;  //not writeable
              status[2] <= 0;
              state <= COMMAND_READ_RESULTS;
              int_state[ds0] <= 1'b1;
              phase <= PHASE_RESPONSE;
            end else begin
              $display("Setting up track info reload");
              m_status[UPD765_MAIN_RQM] <= 0;
              i_command <= COMMAND_RW_DATA_EXEC1;
              state <= COMMAND_RELOAD_TRACKINFO;
            end
          end

          // Añadir logs al COMMAND_RELOAD_TRACKINFO
          COMMAND_RELOAD_TRACKINFO: begin
            $display("COMMAND_RELOAD_TRACKINFO: image_ready=%b, trackinfo_dirty=%b",
                     image_ready[ds0], image_trackinfo_dirty[ds0]);

            if (image_ready[ds0] & image_trackinfo_dirty[ds0]) begin
              $display("Reloading track info");
              //i_rpm_timer[ds0] <= '{ 0, 0 };
              next_weak_sector[ds0] <= 0;
              image_track_offsets_addr <= {pcn[ds0], 1'b0};
              old_hds <= hds;
              hds <= 0;
              buff_wait <= 1;
              state <= COMMAND_RELOAD_TRACKINFO1;
            end else begin
              $display("No need to reload track info, proceeding to i_command=%d", i_command);
              state <= i_command;
            end
          end

          COMMAND_RELOAD_TRACKINFO1:
          if (~buff_wait & ~sd_busy) begin
            if (image_ready[ds0] && image_track_offsets_in) begin
              sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
              sd_rd[ds0] <= 1;
              sd_lba <= image_track_offsets_in[15:1];
              sd_busy <= 1;
              state <= COMMAND_RELOAD_TRACKINFO2;
            end else begin
              image_trackinfo_dirty[ds0] <= 0;
              image_ready[ds0] <= 1;  //rampa
              hds <= old_hds;
              state <= i_command;
            end
          end

          COMMAND_RELOAD_TRACKINFO2:
          if (~sd_busy) begin
            buff_addr <= {image_track_offsets_in[0], 8'h15};  //number of sectors
            buff_wait <= 1;
            state <= COMMAND_RELOAD_TRACKINFO3;
          end

          // We now have the track buffer loaded into buff_addr and offset pointing to num sectors in track

          COMMAND_RELOAD_TRACKINFO3:
          if (~sd_busy & ~buff_wait) begin
            i_current_track_sectors[ds0][hds] <= buff_data_in;
            //i_rpm_time[ds0][hds] <= buff_data_in ? TRACK_TIME/buff_data_in : cycles_time;

            //assume the head position is at the middle of a track after a seek
            i_current_sector_pos[ds0][hds] <= buff_data_in[7:1];

            if (hds == image_sides[ds0]) begin
              image_trackinfo_dirty[ds0] <= 0;
              hds <= old_hds;
              state <= i_command;  // Exit back to COMMAND_READ_ID2 / COMMAND_RW_DATA_EXEC1
            end else begin  //read TrackInfo from the other head if 2 sided
              image_track_offsets_addr <= {pcn[ds0], 1'b1};
              hds <= 1;
              buff_wait <= 1;
              state <= COMMAND_RELOAD_TRACKINFO1;
            end
          end

        endcase  //status
      end
    end
  end

endmodule

module u765_dpram #(
    parameter DATAWIDTH = 8,
    ADDRWIDTH = 12
) (
    input clock,

    input      [ADDRWIDTH-1:0] address_a,
    input      [DATAWIDTH-1:0] data_a,
    input                      wren_a,
    output reg [DATAWIDTH-1:0] q_a,

    input      [ADDRWIDTH-1:0] address_b,
    input      [DATAWIDTH-1:0] data_b,
    input                      wren_b,
    output reg [DATAWIDTH-1:0] q_b
);

  logic [DATAWIDTH-1:0] ram[0:(1<<ADDRWIDTH)-1];

  always_ff @(posedge clock) begin
    if (wren_a) begin
      ram[address_a] <= data_a;
      q_a <= data_a;
    end else begin
      q_a <= ram[address_a];
    end
  end

  always_ff @(posedge clock) begin
    if (wren_b) begin
      ram[address_b] <= data_b;
      q_b <= data_b;
    end else begin
      q_b <= ram[address_b];
    end
  end

endmodule

