module pulsegenerator
#(
   parameter  [ 2:0]  C_SELECT_BIT    = 1,
   parameter          PIPELINE_OUTPUT = 2
)
(
   // ip-sync AV-ST sink
   input              S_AVST_VALID,
   input      [ 7:0]  S_AVST_DATA,
   output reg         S_AVST_READY,

   // Ref signals
   input clk_ref,
   input rst_sys,

   // standard AXI-MM sink
   input 	          S_AXI_ACLK,
   input 	          S_AXI_ARESETN,
   input      [ 3:0]  S_AXI_AWADDR,
   input 	          S_AXI_AWVALID,
   output 	          S_AXI_AWREADY,
   input      [31:0]  S_AXI_WDATA,
   input      [ 3:0]  S_AXI_WSTRB,
   input 	          S_AXI_WVALID,
   output 	          S_AXI_WREADY,
   output     [ 1:0]  S_AXI_BRESP,
   output 	          S_AXI_BVALID,
   input 	          S_AXI_BREADY,
   input      [ 3:0]  S_AXI_ARADDR,
   input 	          S_AXI_ARVALID,
   output 	          S_AXI_ARREADY,
   output     [31:0]  S_AXI_RDATA,
   output     [ 1:0]  S_AXI_RRESP,
   output 	          S_AXI_RVALID,
   input      [ 2:0]  S_AXI_ARPROT,
   input      [ 2:0]  S_AXI_AWPROT,
   input 	          S_AXI_RREADY,

   // output AV-ST source
   output     [ 7:0] M_AVST_DATA,
   output  	         M_AVST_VALID,
   input             M_AVST_READY
);

   //------------------------Register Address Map-------------------
   // 0x000 : Pulse Length
   // 0x004 : IP-Sync Enable

   //------------------------ Parameter ----------------------
   localparam
     // register address map - words
     ADDR_PULSE_LENGTH_LP = 4'h0,
     ADDR_IPSYNC_ENABLE   = 4'h4,

     // write states
     WRIDLE               = 2'd0,
     WRDATA               = 2'd1,
     WRRESP               = 2'd2,
     WRRESET              = 2'd3,

     // read states
     RDIDLE              = 2'd0,
     RDDATA              = 2'd1,
     RDRESET             = 2'd2,

     // ipsync states
     IPSYNC_IDLE         = 1'b0,
     IPSYNC_POLL         = 1'b1,

     // counter states
     CNT_STOP           = 1'b0,
     CNT_CNT            = 1'b1;

   // Global assignments
   wire clk_i =  S_AXI_ACLK;
   wire rst_i = ~S_AXI_ARESETN;

   // registers ip-sync
   reg        rIpSyncState;
   reg [31:0] rIpSyncEnable;
   reg        rIpStart;

   // registers pulse counter
   reg [ 7:0] rPulseCounter;

   // registers valid counter
   reg        rDownCount;
   reg [31:0] rValidCounter;
   reg        rCounterState;

   // sample sequnce done flag
   reg        rDone;
   reg        IpStart;

   // Grey count reference
   reg [3:0] q, rPulseCounter_ref, rPulseCounter_ref_last;
   // pulsecounter reference
   always @(posedge clk_ref) begin
      if(rst_sys) begin
	     rPulseCounter_ref <= 4'b0;
	     q <= 4'b0;
      end else begin
         q <= q + 4'b1;
         rPulseCounter_ref <= {q[3], q[3:1] ^ q[2:0]};
      end
   end

   reg rst_sync_state;
   // pulsecounter continuous downcount, edge detects the reference
   always @(posedge clk_i) begin
      if(rst_i) begin
         rPulseCounter_ref_last <= 4'hF;
         rst_sync_state <= 1'b0;
	     rPulseCounter <= 8'b0;
      end else if(rst_sync_state == 1'b0) begin
         rPulseCounter_ref_last <= rPulseCounter_ref;
         rst_sync_state <= (rPulseCounter_ref == 4'h1 && rPulseCounter_ref_last == 4'h0);
      end else begin
	     rPulseCounter <= rPulseCounter + 8'b1;
      end
   end

   // AXI internal signals
   reg  [ 1:0] rWrState;
   reg  [ 1:0] rWrState_next;
   reg  [11:0] rWrAddr;
   wire [31:0] wmask;
   wire        aw_hs;
   wire        w_hs;

   reg  [ 1:0] rRdState;
   reg  [ 1:0] rRdState_next;
   reg  [31:0] rRdData;
   wire        ar_hs;
   wire [11:0] raddr;

   // AXI-Lite logic

   //------------------------AXI write fsm------------------
   assign S_AXI_AWREADY = (rWrState == WRIDLE);
   assign S_AXI_WREADY  = (rWrState == WRDATA);
   assign S_AXI_BRESP   = 2'b00;  // OKAY
   assign S_AXI_BVALID  = (rWrState == WRRESP);
   assign wmask         = { {8{S_AXI_WSTRB[3]}}, {8{S_AXI_WSTRB[2]}}, {8{S_AXI_WSTRB[1]}}, {8{S_AXI_WSTRB[0]}} };
   assign aw_hs         = S_AXI_AWVALID & S_AXI_AWREADY;
   assign w_hs          = S_AXI_WVALID & S_AXI_WREADY;

   // rWrState
   always @(posedge clk_i) begin
      if (rst_i)
         rWrState <= WRRESET;
      else
         rWrState <= rWrState_next;
   end

   // rWrState_next
   always @(*) begin
      case (rWrState)
         WRIDLE:
            if (S_AXI_AWVALID)
               rWrState_next = WRDATA;
            else
               rWrState_next = WRIDLE;
         WRDATA:
            if (S_AXI_WVALID)
               rWrState_next = WRRESP;
            else
               rWrState_next = WRDATA;
         WRRESP:
            if (S_AXI_BREADY)
               rWrState_next = WRIDLE;
            else
               rWrState_next = WRRESP;
         default:
            rWrState_next = WRIDLE;
         endcase
   end

   // rWrAddr
   always @(posedge clk_i) begin
      if (aw_hs)
            rWrAddr <= S_AXI_AWADDR;
   end

   //------------------------AXI read fsm-------------------
   assign S_AXI_ARREADY = (rRdState == RDIDLE);
   assign S_AXI_RDATA   = rRdData;
   assign S_AXI_RRESP   = 2'b00;  // OKAY
   assign S_AXI_RVALID  = (rRdState == RDDATA);
   assign ar_hs   = S_AXI_ARVALID & S_AXI_ARREADY;
   assign raddr   = S_AXI_ARADDR;

   // rRdState
   always @(posedge clk_i) begin
      if (rst_i)
            rRdState <= RDRESET;
      else
            rRdState <= rRdState_next;
   end

   // rRdState_next
   always @(*) begin
      case (rRdState)
      RDIDLE:
         if (S_AXI_ARVALID)
         rRdState_next = RDDATA;
         else
         rRdState_next = RDIDLE;
      RDDATA:
         if (S_AXI_RREADY & S_AXI_RVALID)
         rRdState_next = RDIDLE;
         else
         rRdState_next = RDDATA;
      default:
         rRdState_next = RDIDLE;
      endcase
   end

   // rdata
   always @(posedge clk_i) begin
      if (ar_hs) begin
         case (raddr)
            ADDR_PULSE_LENGTH_LP:
               rRdData <= 32'hAAAA_AAAA;
            ADDR_IPSYNC_ENABLE:
               rRdData <= rIpSyncEnable;
            default:
               rRdData <= 32'b0;
         endcase
      end
   end

  // register update operations
  always @(posedge clk_i) begin
     if (rst_i) begin
       rValidCounter <= 32'b0;
       rIpSyncEnable <= 32'b0;
     // valid counter  update
     end else begin
        // user enters new sample count
        if (w_hs && (rWrAddr == ADDR_PULSE_LENGTH_LP)) begin
           rValidCounter <= S_AXI_WDATA;
        // downcount
        end else if (rDownCount) begin
           rValidCounter <= rValidCounter - 32'b1;
        end
        // user configures ip-sync
        if ( w_hs && (rWrAddr == ADDR_IPSYNC_ENABLE)) begin
           rIpSyncEnable <= S_AXI_WDATA;
        end
     end
  end

   // ip-start AV-ST sink - disabled, TODO - add logic
   always@(posedge clk_i) begin
      if(rst_i) begin
         IpStart      <= 1'b0;
         S_AVST_READY <= 1'b0;
      end else begin
         S_AVST_READY <= 1'b1;
         if(S_AVST_VALID)
            IpStart <= S_AVST_DATA[0];
         else
            IpStart <= 1'b0;
      end
   end

   // Trigger set by sample sequence start logic
   // ip-sync state machine
   always@(posedge clk_i) begin
      if(rst_i) begin
         rIpSyncState <= IPSYNC_IDLE;
         rIpStart     <= 1'b0;
      end else if(rIpSyncEnable == 32'b0) begin
         rIpSyncState <= IPSYNC_IDLE;
         rIpStart     <= 1'b1;
      end else
         case(rIpSyncState)
            IPSYNC_IDLE:
               // spin until valid counter is initialized
               if( (rValidCounter != 32'd0) && (M_AVST_READY == 1'b1)) begin
                  rIpSyncState <= IPSYNC_POLL;
                  rIpStart     <= IpStart;
               end else begin
                  rIpSyncState <= IPSYNC_IDLE;
                  rIpStart     <= 1'b0;
               end
            IPSYNC_POLL:
               // spin until target ip triggers sample sequence
               if(IpStart) begin
                  rIpSyncState <= IPSYNC_POLL;
                  rIpStart     <= 1'b1;
               // return to idle when sample sequnce done flags
               end else if(rDone) begin
                  rIpSyncState <= IPSYNC_IDLE;
                  rIpStart     <= 1'b0;
               end
         endcase
   end

   // valid counter alignment state machine
   always@(posedge clk_i) begin
      if(rst_i) begin
         rCounterState <= CNT_STOP;
         rDownCount    <= 1'b0;
         rDone         <= 1'b0;
      end else begin
         case(rCounterState)
            CNT_STOP:
               // enable when pg is primed, on a rising transition, and ip-start is set (default)
               if( (rValidCounter != 32'd0) && (rPulseCounter[1:0] == 2'b1) && (rIpStart == 1'b1) && (M_AVST_READY == 1'b1)) begin
                  // start valid counter
                  rCounterState <= CNT_CNT;
                  rDownCount    <= 1'b1;
                  rDone         <= 1'b0;
               end else begin
                  // spin until start condition
                  rCounterState <= CNT_STOP;
                  rDownCount    <= 1'b0;
                  rDone         <= 1'b0;
               end
            CNT_CNT:
               if(rValidCounter == 32'd1) begin
                  // last sample
                  rCounterState <= CNT_STOP;
                  rDownCount    <= 1'b0;
                  rDone         <= 1'b1;
               end else begin
                  // main sample sequence
                  rCounterState <= CNT_CNT;
                  rDownCount    <= 1'b1;
                  rDone         <= 1'b0;
               end
         endcase
      end
   end

   reg [7:0]
		M_AVST_DATA_P [0:PIPELINE_OUTPUT-1];

   reg [PIPELINE_OUTPUT-1:0]
		M_AVST_VALID_P;

   // pipeline output AV-ST interface
   always@(posedge clk_i) begin
      if(rst_i) begin
         M_AVST_VALID_P[0] <= 1'b0;
         M_AVST_DATA_P[0]  <= 8'b0;
      end else begin
         M_AVST_VALID_P[0] <= (~rDone) & rDownCount;
         M_AVST_DATA_P[0]  <= ((8'b1 << (C_SELECT_BIT[2:0])) & rPulseCounter) >> (C_SELECT_BIT[2:0]);
      end
   end

   // output register stages
   genvar i;
   generate
      for (i=1; i < PIPELINE_OUTPUT; i = i + 1) begin : pipe_out
         always@(posedge clk_i) begin
            M_AVST_VALID_P[i]  <= rst_i ? 1'b0 : M_AVST_VALID_P[i-1];
            M_AVST_DATA_P[i]   <= rst_i ? 8'b0 :M_AVST_DATA_P[i-1];
         end
      end
   endgenerate

   assign M_AVST_VALID = M_AVST_VALID_P[PIPELINE_OUTPUT-1];
   assign M_AVST_DATA  = M_AVST_DATA_P[PIPELINE_OUTPUT-1];

endmodule
