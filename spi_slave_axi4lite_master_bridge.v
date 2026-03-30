
`timescale 1ns / 1ps

module spi_slave_axi4lite_master_bridge #(
    parameter integer FRAME_W = 24,
    parameter integer DATA_W  = 16,
    parameter integer ADDR_W  = 32
)(
    input  wire              aclk,
    input  wire              aresetn,

    // SPI slave ports
    input  wire              spi_sclk,
    input  wire              spi_cs_n,
    input  wire              spi_mosi,
    output reg               spi_miso,

    // AXI4-Lite master ports
    output reg  [ADDR_W-1:0] m_axi_awaddr,
    output reg               m_axi_awvalid,
    input  wire              m_axi_awready,

    output reg  [DATA_W-1:0] m_axi_wdata,
    output reg  [1:0]        m_axi_wstrb,
    output reg               m_axi_wvalid,
    input  wire              m_axi_wready,

    input  wire [1:0]        m_axi_bresp,
    input  wire              m_axi_bvalid,
    output reg               m_axi_bready,

    output reg  [ADDR_W-1:0] m_axi_araddr,
    output reg               m_axi_arvalid,
    input  wire              m_axi_arready,

    input  wire [DATA_W-1:0] m_axi_rdata,
    input  wire [1:0]        m_axi_rresp,
    input  wire              m_axi_rvalid,
    output reg               m_axi_rready
);


//  FSM states
localparam [2:0]
    ST_IDLE      = 3'd0,
    ST_DECODE    = 3'd1,
    ST_AXI_WRITE = 3'd2,
    ST_AXI_WRESP = 3'd3,
    ST_AXI_READ  = 3'd4,
    ST_AXI_RDATA = 3'd5,
    ST_DONE      = 3'd6;

reg [2:0] state;


//  2-FF synchronisers
reg [1:0] sclk_sync, csn_sync, mosi_sync;

always @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        sclk_sync <= 2'b00;
        csn_sync  <= 2'b11;
        mosi_sync <= 2'b00;
    end else begin
        sclk_sync <= {sclk_sync[0], spi_sclk};
        csn_sync  <= {csn_sync[0],  spi_cs_n};
        mosi_sync <= {mosi_sync[0], spi_mosi};
    end
end

wire sclk_rise   =  sclk_sync[0] & ~sclk_sync[1];
wire sclk_fall   = ~sclk_sync[0] &  sclk_sync[1];
wire cs_active   = ~csn_sync[1];
wire cs_assert   = ~csn_sync[0] &  csn_sync[1];   // CS just went LOW
wire cs_deassert =  csn_sync[0] & ~csn_sync[1];   // CS just went HIGH


//  SPI receive shift register
reg [FRAME_W-1:0] rx_frame;
reg [FRAME_W-1:0] decoded_frame;
reg [4:0]         bit_cnt;
reg               frame_done;

always @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        rx_frame      <= {FRAME_W{1'b0}};
        decoded_frame <= {FRAME_W{1'b0}};
        bit_cnt       <= 5'd0;
        frame_done    <= 1'b0;
    end else begin
        frame_done <= 1'b0;

        if (!cs_active) begin
            bit_cnt  <= 5'd0;
            rx_frame <= {FRAME_W{1'b0}};
        end else if (sclk_rise) begin
            rx_frame <= {rx_frame[FRAME_W-2:0], mosi_sync[1]};
            bit_cnt  <= bit_cnt + 5'd1;

            if (bit_cnt == FRAME_W - 1) begin
                decoded_frame <= {rx_frame[FRAME_W-2:0], mosi_sync[1]};
                frame_done    <= 1'b1;
                bit_cnt       <= 5'd0;
            end
        end
    end
end


//  Decoded request registers
reg               req_rw;
reg [6:0]         req_addr;
reg [DATA_W-1:0]  req_data;


//  rdata latch and control flags
reg [DATA_W-1:0] rdata_latch;
reg              rdata_valid;
reg              pending_read;
reg              consume_rdata;  

always @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        rdata_latch  <= {DATA_W{1'b0}};
        rdata_valid  <= 1'b0;
        pending_read <= 1'b0;
    end else begin
        if (m_axi_rvalid && m_axi_rready) begin
            rdata_latch  <= m_axi_rdata;
            rdata_valid  <= 1'b1;
            pending_read <= 1'b1;
        end
        if (consume_rdata) begin
            rdata_valid  <= 1'b0;
            pending_read <= 1'b0;
        end
    end
end

//  MISO shift register and output
reg [FRAME_W-1:0] miso_shift;

always @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        spi_miso      <= 1'b0;
        miso_shift    <= {FRAME_W{1'b0}};
        consume_rdata <= 1'b0;
    end else begin
        consume_rdata <= 1'b0;   // default: no consume

        if (cs_assert) begin
            if (rdata_valid) begin
                // Response frame: { 1'b0, req_addr[6:0], rdata_latch[15:0] }
                spi_miso   <= 1'b0;
                // Pre-shift: miso_shift[23] = frame[22] for first falling edge
                miso_shift <= {req_addr, rdata_latch, 1'b0};
                consume_rdata <= 1'b1;
            end else begin
                spi_miso   <= 1'b0;
                miso_shift <= {FRAME_W{1'b0}};
            end

        end else if (sclk_fall && cs_active) begin
            spi_miso   <= miso_shift[FRAME_W-1];
            miso_shift <= {miso_shift[FRAME_W-2:0], 1'b0};

        end else if (!cs_active) begin
            spi_miso <= 1'b0;
        end
    end
end

reg aw_done, w_done;

//  Main FSM
always @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        state         <= ST_IDLE;
        req_rw        <= 1'b0;
        req_addr      <= 7'd0;
        req_data      <= {DATA_W{1'b0}};
        m_axi_awaddr  <= {ADDR_W{1'b0}};
        m_axi_awvalid <= 1'b0;
        m_axi_wdata   <= {DATA_W{1'b0}};
        m_axi_wstrb   <= 2'b11;
        m_axi_wvalid  <= 1'b0;
        m_axi_bready  <= 1'b0;
        m_axi_araddr  <= {ADDR_W{1'b0}};
        m_axi_arvalid <= 1'b0;
        m_axi_rready  <= 1'b0;
        aw_done       <= 1'b0;
        w_done        <= 1'b0;
    end else begin

        case (state)

            //Wait for complete SPI frame
            ST_IDLE: begin
                aw_done <= 1'b0;
                w_done  <= 1'b0;
                if (frame_done)
                    state <= ST_DECODE;
            end

            // Decode; detect DUMMY frame (BUG A FIX)
            ST_DECODE: begin
                req_rw   <= decoded_frame[FRAME_W-1];
                req_addr <= decoded_frame[FRAME_W-2 : DATA_W];
                req_data <= decoded_frame[DATA_W-1  : 0];

                if (pending_read) begin
                    state <= ST_DONE;
                end else if (decoded_frame[FRAME_W-1]) begin
                    state <= ST_AXI_WRITE;
                end else begin
                    state <= ST_AXI_READ;
                end
            end

            ST_AXI_WRITE: begin
                if (!aw_done) begin
                    m_axi_awaddr  <= {{(ADDR_W-7){1'b0}}, req_addr};
                    m_axi_awvalid <= 1'b1;
                end
                if (!w_done) begin
                    m_axi_wdata  <= req_data;
                    m_axi_wstrb  <= 2'b11;
                    m_axi_wvalid <= 1'b1;
                end

                if (m_axi_awvalid && m_axi_awready) begin
                    m_axi_awvalid <= 1'b0;
                    aw_done       <= 1'b1;
                end
                if (m_axi_wvalid && m_axi_wready) begin
                    m_axi_wvalid <= 1'b0;
                    w_done       <= 1'b1;
                end

                if ((aw_done || (m_axi_awvalid && m_axi_awready)) &&
                    (w_done  || (m_axi_wvalid  && m_axi_wready ))) begin
                    m_axi_bready <= 1'b1;
                    state        <= ST_AXI_WRESP;
                end
            end

            ST_AXI_WRESP: begin
                if (m_axi_bvalid && m_axi_bready) begin
                    m_axi_bready <= 1'b0;
                    state        <= ST_DONE;
                end
            end

            ST_AXI_READ: begin
                m_axi_araddr  <= {{(ADDR_W-7){1'b0}}, req_addr};
                m_axi_arvalid <= 1'b1;

                if (m_axi_arvalid && m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                    state         <= ST_AXI_RDATA;
                end
            end

            ST_AXI_RDATA: begin
                if (m_axi_rvalid && m_axi_rready) begin
                    m_axi_rready <= 1'b0;
                    state        <= ST_DONE;
                    
                end
            end

            ST_DONE: begin
                if (cs_deassert || !cs_active)
                    state <= ST_IDLE;
            end

            default: state <= ST_IDLE;

        endcase
    end
end

endmodule
