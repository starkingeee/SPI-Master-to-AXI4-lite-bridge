

`timescale 1ns / 1ps

module tb_spi_slave_axi4lite_master_bridge;

//Parameters
localparam FRAME_W    = 24;
localparam DATA_W     = 16;
localparam ADDR_W     = 32;
localparam CLK_PERIOD = 10;    // 100 MHz aclk
localparam SPI_HALF   = 40;    // SPI half-period (12.5 MHz)

// DUT signals
reg               aclk, aresetn;
reg               spi_sclk, spi_cs_n, spi_mosi;
wire              spi_miso;

wire [ADDR_W-1:0] m_axi_awaddr;
wire              m_axi_awvalid;
reg               m_axi_awready;
wire [DATA_W-1:0] m_axi_wdata;
wire [1:0]        m_axi_wstrb;
wire              m_axi_wvalid;
reg               m_axi_wready;
reg  [1:0]        m_axi_bresp;
reg               m_axi_bvalid;
wire              m_axi_bready;
wire [ADDR_W-1:0] m_axi_araddr;
wire              m_axi_arvalid;
reg               m_axi_arready;
reg  [DATA_W-1:0] m_axi_rdata;
reg  [1:0]        m_axi_rresp;
reg               m_axi_rvalid;
wire              m_axi_rready;

// DUT 
spi_slave_axi4lite_master_bridge #(
    .FRAME_W(FRAME_W), .DATA_W(DATA_W), .ADDR_W(ADDR_W)
) dut (
    .aclk(aclk), .aresetn(aresetn),
    .spi_sclk(spi_sclk), .spi_cs_n(spi_cs_n),
    .spi_mosi(spi_mosi), .spi_miso(spi_miso),
    .m_axi_awaddr(m_axi_awaddr), .m_axi_awvalid(m_axi_awvalid),
    .m_axi_awready(m_axi_awready),
    .m_axi_wdata(m_axi_wdata),   .m_axi_wstrb(m_axi_wstrb),
    .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
    .m_axi_bresp(m_axi_bresp),   .m_axi_bvalid(m_axi_bvalid),
    .m_axi_bready(m_axi_bready),
    .m_axi_araddr(m_axi_araddr), .m_axi_arvalid(m_axi_arvalid),
    .m_axi_arready(m_axi_arready),
    .m_axi_rdata(m_axi_rdata),   .m_axi_rresp(m_axi_rresp),
    .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready)
);

// Clock 
initial aclk = 0;
always #(CLK_PERIOD/2) aclk = ~aclk;

// AXI Slave model
// Responds in 2-cycle delay. Write → BRESP OKAY. Read → returns m_axi_rdata.

initial begin
    m_axi_awready=0; m_axi_wready=0; m_axi_bresp=0; m_axi_bvalid=0;
    m_axi_arready=0; m_axi_rdata=0;  m_axi_rresp=0; m_axi_rvalid=0;
end

// Write address accept
always @(posedge aclk) begin
    if (m_axi_awvalid && !m_axi_awready) begin
        repeat(2) @(posedge aclk);
        m_axi_awready <= 1'b1;
        @(posedge aclk);
        m_axi_awready <= 1'b0;
    end
end

// Write data accept
always @(posedge aclk) begin
    if (m_axi_wvalid && !m_axi_wready) begin
        repeat(2) @(posedge aclk);
        m_axi_wready <= 1'b1;
        @(posedge aclk);
        m_axi_wready <= 1'b0;
    end
end

// Write response
always @(posedge aclk) begin
    if (m_axi_bready && !m_axi_bvalid) begin
        repeat(2) @(posedge aclk);
        m_axi_bvalid <= 1'b1; m_axi_bresp <= 2'b00;
        @(posedge aclk);
        m_axi_bvalid <= 1'b0;
    end
end

//  Read address accept + data return
always @(posedge aclk) begin
    if (m_axi_arvalid && !m_axi_arready) begin
        repeat(2) @(posedge aclk);
        m_axi_arready <= 1'b1;
        @(posedge aclk);
        m_axi_arready <= 1'b0;
        repeat(2) @(posedge aclk);
        m_axi_rvalid <= 1'b1;
        @(posedge aclk);
        if (m_axi_rready) m_axi_rvalid <= 1'b0;
    end
end

// SPI frame 
task spi_transfer;
    input  [FRAME_W-1:0] tx;
    output [FRAME_W-1:0] rx;
    integer i;
    reg [FRAME_W-1:0] cap;
    begin
        cap      = 0;
        spi_cs_n = 0;
        spi_sclk = 0;
        #(SPI_HALF);

        for (i = FRAME_W-1; i >= 0; i = i-1) begin
            spi_mosi = tx[i];
            #(SPI_HALF);
            spi_sclk = 1;               
            #(SPI_HALF/2);
            cap = {cap[FRAME_W-2:0], spi_miso};  
            #(SPI_HALF/2);
            spi_sclk = 0;               
        end

        #(SPI_HALF);
        spi_cs_n = 1;                  
        spi_mosi = 0;
        #(SPI_HALF * 4);               
        rx = cap;
    end
endtask

//AXI bus monitor
always @(posedge aclk) begin
    if (m_axi_awvalid && m_axi_awready)
        $display("  [AXI] Write Addr  accepted : 0x%08X", m_axi_awaddr);
    if (m_axi_wvalid  && m_axi_wready)
        $display("  [AXI] Write Data  accepted : 0x%04X  strb=%02b",
                  m_axi_wdata, m_axi_wstrb);
    if (m_axi_bvalid  && m_axi_bready)
        $display("  [AXI] Write Resp  received : BRESP=%02b", m_axi_bresp);
    if (m_axi_arvalid && m_axi_arready)
        $display("  [AXI] Read  Addr  accepted : 0x%08X", m_axi_araddr);
    if (m_axi_rvalid  && m_axi_rready)
        $display("  [AXI] Read  Data  received : 0x%04X  RRESP=%02b",
                  m_axi_rdata, m_axi_rresp);
end


reg [FRAME_W-1:0] tx_frame, rx_frame;

initial begin
    spi_sclk = 0; spi_cs_n = 1; spi_mosi = 0;
    aresetn  = 0;
    repeat(10) @(posedge aclk);
    aresetn = 1;
    repeat(5)  @(posedge aclk);

    // TEST 1: SPI Write  addr=0x01  data=0x1234
    $display("──────────────────────────────────────────");
    $display("TEST 1: SPI Write  (addr=0x01, data=0x1234)");
    tx_frame = {1'b1, 7'h01, 16'h1234};
    $display("  TX frame = 0x%06X", tx_frame);
    spi_transfer(tx_frame, rx_frame);
    #(CLK_PERIOD * 30);
    $display("  PASS if [AXI] lines above show addr=0x01, data=0x1234");

    // TEST 2: SPI Read   addr=0x02  (TWO-FRAME PROTOCOL)
    $display("──────────────────────────────────────────");
    $display("TEST 2: SPI Read  (addr=0x02, expected rdata=0xABCD)");

    // Pre-load slave response BEFORE issuing the read frame
    m_axi_rdata = 16'hABCD;

    // Frame A: READ request
    tx_frame = {1'b0, 7'h02, 16'h0000};
    $display("  Frame A (READ request) TX = 0x%06X", tx_frame);
    spi_transfer(tx_frame, rx_frame);
    #(CLK_PERIOD * 20);

    // Frame B: DUMMY frame - master clocks out MISO response
    tx_frame = 24'h000000;
    $display("  Frame B (DUMMY)        TX = 0x%06X", tx_frame);
    spi_transfer(tx_frame, rx_frame);

    $display("  MISO rx_frame = 0x%06X  (data field = 0x%04X)",
              rx_frame, rx_frame[DATA_W-1:0]);

    if (rx_frame[DATA_W-1:0] == 16'hABCD)
        $display("  PASS: Got 0xABCD on MISO");
    else
        $display("  FAIL: Expected 0xABCD, got 0x%04X", rx_frame[DATA_W-1:0]);

    // TEST 3: Back-to-back writes
    $display("──────────────────────────────────────────");
    $display("TEST 3: Back-to-back writes");

    tx_frame = {1'b1, 7'h10, 16'hDEAD};
    $display("  Write addr=0x10 data=0xDEAD");
    spi_transfer(tx_frame, rx_frame);
    #(CLK_PERIOD * 20);

    tx_frame = {1'b1, 7'h7F, 16'hBEEF};
    $display("  Write addr=0x7F data=0xBEEF");
    spi_transfer(tx_frame, rx_frame);
    #(CLK_PERIOD * 20);

    // TEST 4: Two consecutive reads, different addresses
    $display("──────────────────────────────────────────");
    $display("TEST 4: Two consecutive reads");

    // Read addr=0x05, slave returns 0x1111
    m_axi_rdata = 16'h1111;
    tx_frame    = {1'b0, 7'h05, 16'h0000};
    $display("  Frame A: READ addr=0x05");
    spi_transfer(tx_frame, rx_frame);
    #(CLK_PERIOD * 20);
    tx_frame = 24'h000000;
    spi_transfer(tx_frame, rx_frame);
    $display("  Frame B MISO = 0x%04X  (expect 0x1111)", rx_frame[DATA_W-1:0]);
    if (rx_frame[DATA_W-1:0] == 16'h1111)
        $display("  PASS");
    else
        $display("  FAIL: got 0x%04X", rx_frame[DATA_W-1:0]);

    // Read addr=0x0A, slave returns 0x2222
    m_axi_rdata = 16'h2222;
    tx_frame    = {1'b0, 7'h0A, 16'h0000};
    $display("  Frame A: READ addr=0x0A");
    spi_transfer(tx_frame, rx_frame);
    #(CLK_PERIOD * 20);
    tx_frame = 24'h000000;
    spi_transfer(tx_frame, rx_frame);
    $display("  Frame B MISO = 0x%04X  (expect 0x2222)", rx_frame[DATA_W-1:0]);
    if (rx_frame[DATA_W-1:0] == 16'h2222)
        $display("  PASS");
    else
        $display("  FAIL: got 0x%04X", rx_frame[DATA_W-1:0]);

    $display("──────────────────────────────────────────");
    $display("All tests complete.");
    #(CLK_PERIOD * 20);
    $finish;
end

endmodule
