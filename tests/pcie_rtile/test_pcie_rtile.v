`resetall
`timescale 1ns / 1ns
`default_nettype none

module test_pcie_rtile #
(
    parameter SEG_COUNT = 1,
    parameter SEG_DATA_WIDTH = 256,
    parameter SEG_HDR_WIDTH = 128,
    parameter SEG_PRFX_WIDTH = 32,
    parameter SEG_DATA_PAR_WIDTH = 8,
    parameter SEG_HDR_PAR_WIDTH = 4,
    parameter SEG_PRFX_PAR_WIDTH = 1,
    parameter SEG_EMPTY_WIDTH = 3,
    parameter SEG_BAR_WIDTH = 3
)
(
    // Clock and reset
    inout  wire                                     reset_status,
    inout  wire                                     reset_status_n,
    inout  wire                                     coreclkout_hip,
    inout  wire                                     refclk0,
    inout  wire                                     refclk1,
    inout  wire                                     pin_perst_n,

    // RX interface
    // rx bus rx_source = None
    inout  wire [SEG_COUNT*SEG_DATA_WIDTH-1:0]      rx_st_data,
    inout  wire [SEG_COUNT*SEG_HDR_WIDTH-1:0]       rx_st_hdr,
    inout  wire [SEG_COUNT*SEG_PRFX_WIDTH-1:0]      rx_st_prefix,
    inout  wire [SEG_COUNT-1:0]                     rx_st_sop,
    inout  wire [SEG_COUNT-1:0]                     rx_st_eop,
    inout  wire [SEG_COUNT-1:0]                     rx_st_dvalid,
    inout  wire [SEG_COUNT-1:0]                     rx_st_hvalid,
    inout  wire [SEG_COUNT-1:0]                     rx_st_pvalid,
    inout  wire                                     rx_st_ready,
    inout  wire [SEG_COUNT*SEG_BAR_WIDTH-1:0]       rx_st_bar,
    inout  wire [SEG_COUNT*SEG_DATA_PAR_WIDTH-1:0]  rx_st_data_par,
    inout  wire [SEG_COUNT*SEG_HDR_PAR_WIDTH-1:0]   rx_st_hdr_par,
    inout  wire [SEG_COUNT*SEG_PRFX_PAR_WIDTH-1:0]  rx_st_prefix_par,
    inout  wire [SEG_COUNT*SEG_EMPTY_WIDTH-1:0]     rx_st_empty,
    inout  wire [SEG_COUNT*1:0]                     rx_st_pt_par,
    inout  wire [SEG_COUNT-1:0]                     rx_st_vfactive,
    inout  wire [SEG_COUNT*11-1:0]                  rx_st_vfnum,
    inout  wire [SEG_COUNT*3-1:0]                   rx_st_pfnum,

    inout  wire [2:0]                               rx_st_hcrdt_update,
    inout  wire [5:0]                               rx_st_hcrdt_update_cnt,
    inout  wire [2:0]                               rx_st_hcrdt_init,
    inout  wire [2:0]                               rx_st_hcrdt_init_ack,
    inout  wire [2:0]                               rx_st_dcrdt_update,
    inout  wire [11:0]                              rx_st_dcrdt_update_cnt,
    inout  wire [2:0]                               rx_st_dcrdt_init,
    inout  wire [2:0]                               rx_st_dcrdt_init_ack,

    // TX interface
    // tx bus tx_sink = None
    inout  wire [SEG_COUNT*SEG_DATA_WIDTH-1:0]      tx_st_data,
    inout  wire [SEG_COUNT*SEG_HDR_WIDTH-1:0]       tx_st_hdr,
    inout  wire [SEG_COUNT*SEG_PRFX_WIDTH-1:0]      tx_st_prefix,
    inout  wire [SEG_COUNT-1:0]                     tx_st_sop,
    inout  wire [SEG_COUNT-1:0]                     tx_st_eop,
    inout  wire [SEG_COUNT-1:0]                     tx_st_dvalid,
    inout  wire [SEG_COUNT-1:0]                     tx_st_hvalid,
    inout  wire [SEG_COUNT-1:0]                     tx_st_pvalid,
    inout  wire [SEG_COUNT*SEG_DATA_PAR_WIDTH-1:0]  tx_st_data_par,
    inout  wire [SEG_COUNT*SEG_HDR_PAR_WIDTH-1:0]   tx_st_hdr_par,
    inout  wire [SEG_COUNT*SEG_PRFX_PAR_WIDTH-1:0]  tx_st_prefix_par,
    inout  wire                                     tx_st_ready,
    inout  wire                                     tx_ehp_deallocate_empty,

    inout  wire [2:0]                               tx_st_hcrdt_update,
    inout  wire [5:0]                               tx_st_hcrdt_update_cnt,
    inout  wire [2:0]                               tx_st_hcrdt_init,
    inout  wire [2:0]                               tx_st_hcrdt_init_ack,
    inout  wire [2:0]                               tx_st_dcrdt_update,
    inout  wire [11:0]                              tx_st_dcrdt_update_cnt,
    inout  wire [2:0]                               tx_st_dcrdt_init,
    inout  wire [2:0]                               tx_st_dcrdt_init_ack,

    // Power management and hard IP status interface
    inout  wire                                     link_up,
    inout  wire                                     dl_up,
    inout  wire                                     surprise_down_err,
    inout  wire [5:0]                               ltssm_state,
    inout  wire [2:0]                               pm_state,
    inout  wire [31:0]                              pm_dstate,
    inout  wire [7:0]                               apps_pm_xmt_pme,
    inout  wire [7:0]                               app_req_retry_en,

    // Interrupt interface
    inout  wire [7:0]                               app_int,
    inout  wire [2:0]                               msi_pnd_func,
    inout  wire [7:0]                               msi_pnd_byte,
    inout  wire [1:0]                               msi_pnd_addr,

    // Error interface
    inout  wire                                     serr_out,
    inout  wire                                     hip_enter_err_mode,
    inout  wire                                     app_err_valid,
    inout  wire [31:0]                              app_err_hdr,
    inout  wire [12:0]                              app_err_info,
    inout  wire [2:0]                               app_err_pfnum,

    // Completion timeout interface
    inout  wire                                     cpl_timeout,
    inout  wire                                     cpl_timeout_avmm_clk,
    inout  wire [2:0]                               cpl_timeout_avmm_address,
    inout  wire                                     cpl_timeout_avmm_read,
    inout  wire [7:0]                               cpl_timeout_avmm_readdata,
    inout  wire                                     cpl_timeout_avmm_readdatavalid,
    inout  wire                                     cpl_timeout_avmm_write,
    inout  wire [7:0]                               cpl_timeout_avmm_writedata,
    inout  wire                                     cpl_timeout_avmm_waitrequest,

    // Configuration output
    inout  wire [2:0]                               tl_cfg_func,
    inout  wire [4:0]                               tl_cfg_add,
    inout  wire [15:0]                              tl_cfg_ctl,
    inout  wire                                     dl_timer_update,

    // Configuration intercept interface
    inout  wire                                     cii_req,
    inout  wire                                     cii_hdr_poisoned,
    inout  wire [3:0]                               cii_hdr_first_be,
    inout  wire [2:0]                               cii_pfnum,
    inout  wire                                     cii_wr_vf_active,
    inout  wire [10:0]                              cii_vf_num,
    inout  wire                                     cii_wr,
    inout  wire [9:0]                               cii_addr,
    inout  wire [31:0]                              cii_dout,
    inout  wire                                     cii_override_en,
    inout  wire [31:0]                              cii_override_din,
    inout  wire                                     cii_halt,

    // Hard IP reconfiguration interface
    inout  wire                                     hip_reconfig_clk,
    inout  wire [20:0]                              hip_reconfig_address, // 32?
    inout  wire                                     hip_reconfig_read,
    inout  wire [7:0]                               hip_reconfig_readdata,
    inout  wire                                     hip_reconfig_readdatavalid,
    inout  wire                                     hip_reconfig_write,
    inout  wire [7:0]                               hip_reconfig_writedata,
    inout  wire                                     hip_reconfig_waitrequest,

    // Page request service
    inout  wire                                     prs_event_valid,
    inout  wire [2:0]                               prs_event_func,
    inout  wire [1:0]                               prs_event,

    // SR-IOV (VF error)
    inout  wire                                     vf_err_ur_posted_s0,
    inout  wire                                     vf_err_ur_posted_s1,
    inout  wire                                     vf_err_ur_posted_s2,
    inout  wire                                     vf_err_ur_posted_s3,
    inout  wire [2:0]                               vf_err_pfnum_s0,
    inout  wire [2:0]                               vf_err_pfnum_s1,
    inout  wire [2:0]                               vf_err_pfnum_s2,
    inout  wire [2:0]                               vf_err_pfnum_s3,
    inout  wire                                     vf_err_ca_postedreq_s0,
    inout  wire                                     vf_err_ca_postedreq_s1,
    inout  wire                                     vf_err_ca_postedreq_s2,
    inout  wire                                     vf_err_ca_postedreq_s3,
    inout  wire [10:0]                              vf_err_vf_num_s0,
    inout  wire [10:0]                              vf_err_vf_num_s1,
    inout  wire [10:0]                              vf_err_vf_num_s2,
    inout  wire [10:0]                              vf_err_vf_num_s3,
    inout  wire                                     vf_err_poisonedwrreq_s0,
    inout  wire                                     vf_err_poisonedwrreq_s1,
    inout  wire                                     vf_err_poisonedwrreq_s2,
    inout  wire                                     vf_err_poisonedwrreq_s3,
    inout  wire                                     vf_err_poisonedcompl_s0,
    inout  wire                                     vf_err_poisonedcompl_s1,
    inout  wire                                     vf_err_poisonedcompl_s2,
    inout  wire                                     vf_err_poisonedcompl_s3,
    inout  wire [2:0]                               user_vfnonfatalmsg_pfnum,
    inout  wire [10:0]                              user_vfnonfatalmsg_vfnum,
    inout  wire                                     user_sent_vfnonfatalmsg,
    inout  wire                                     vf_err_overflow,

    // FLR
    inout  wire [7:0]                               flr_rcvd_pf,
    inout  wire                                     flr_rcvd_vf,
    inout  wire [2:0]                               flr_rcvd_pf_num,
    inout  wire [10:0]                              flr_rcvd_vf_num,
    inout  wire [7:0]                               flr_completed_pf,
    inout  wire                                     flr_completed_vf,
    inout  wire [2:0]                               flr_completed_pf_num,
    inout  wire [10:0]                              flr_completed_vf_num,

    // VirtIO
    inout  wire                                     virtio_pcicfg_vfaccess,
    inout  wire [10:0]                              virtio_pcicfg_vfnum,
    inout  wire [2:0]                               virtio_pcicfg_pfnum,
    inout  wire [7:0]                               virtio_pcicfg_bar,
    inout  wire [31:0]                              virtio_pcicfg_length,
    inout  wire [31:0]                              virtio_pcicfg_baroffset,
    inout  wire [31:0]                              virtio_pcicfg_cfgdata,
    inout  wire                                     virtio_pcicfg_cfgwr,
    inout  wire                                     virtio_pcicfg_cfgrd,
    inout  wire [10:0]                              virtio_pcicfg_appvfnum,
    inout  wire [2:0]                               virtio_pcicfg_apppfnum,
    inout  wire                                     virtio_pcicfg_rdack,
    inout  wire [3:0]                               virtio_pcicfg_rdbe,
    inout  wire [31:0]                              virtio_pcicfg_data
);

endmodule

`resetall
