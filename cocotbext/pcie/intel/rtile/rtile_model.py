"""

Copyright (c) 2022 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

"""

from .interface import RTilePcieFrame, RTilePcieSource, RTilePcieSink
from cocotbext.pcie.core.dllp import FcType
from cocotbext.pcie.core.tlp import Tlp, TlpType
from cocotbext.pcie.core.utils import PcieId
from cocotbext.pcie.core.caps import AerExtendedCapability, PcieExtendedCapability
from cocotbext.pcie.core.caps import MsiCapability, MsixCapability
from cocotbext.pcie.core import Device, Endpoint, __version__
from cocotb.triggers import Event, RisingEdge, FallingEdge, Timer, First
from cocotb.queue import Queue
from cocotb.clock import Clock
import cocotb
import logging


valid_configs = [
    # speed, links, width, freq
    (3,  4, 128, 250.0e6),
    (3,  8, 256, 250.0e6),
    (3, 16, 256, 250.0e6),
    (3, 16, 512, 250.0e6),
    (4,  4, 128, 350.0e6),
    (4,  4, 128, 400.0e6),
    (4,  4, 128, 450.0e6),
    (4,  4, 128, 500.0e6),
    (4,  8, 256, 175.0e6),
    (4,  8, 256, 200.0e6),
    (4,  8, 256, 225.0e6),
    (4,  8, 256, 250.0e6),
    (4,  8, 256, 350.0e6),
    (4,  8, 256, 400.0e6),
    (4,  8, 256, 450.0e6),
    (4,  8, 256, 500.0e6),
    (4,  8, 512, 175.0e6),
    (4,  8, 512, 200.0e6),
    (4,  8, 512, 225.0e6),
    (4,  8, 512, 250.0e6),
    (4, 16, 512, 175.0e6),
    (4, 16, 512, 200.0e6),
    (4, 16, 512, 225.0e6),
    (4, 16, 512, 250.0e6),
    (4, 16, 512, 350.0e6),
    (4, 16, 512, 400.0e6),
    (4, 16, 512, 450.0e6),
    (4, 16, 512, 500.0e6),
    (5, 16, 1024, 500.0e6),
]


class RTilePcieFunction(Endpoint):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # PCIe capabilities
        self.register_capability(self.pm_cap, offset=0x10)

        self.msi_cap = MsiCapability()
        self.msi_cap.msi_64bit_address_capable = 1
        self.msi_cap.msi_per_vector_mask_capable = 0
        self.register_capability(self.msi_cap, offset=0x14)

        self.msix_cap = MsixCapability()
        self.register_capability(self.msix_cap, offset=0x1c)

        self.register_capability(self.pcie_cap, offset=0x2c)

        # PCIe extended capabilities
        self.aer_ext_cap = AerExtendedCapability()
        self.register_capability(self.aer_ext_cap, offset=0x40)

        # VC 0x52
        # ARI 0x5e

        self.pcie_ext_cap = PcieExtendedCapability()
        self.register_capability(self.pcie_ext_cap, offset=0x62)

        # PHY16 0x6e
        # LM 0x7a
        # SRIOV 0x8c
        # TPH 0x9c
        # ATS 0xbf
        # ACS 0xc3
        # PRS 0xc6
        # LTR 0xcb
        # PASID 0xcc
        # VSEC (RAS D.E.S.) 0xce
        # DL 0x11c
        # VSEC (Intel) 0x340


def init_signal(sig, width=None, initval=None):
    if sig is None:
        return None
    if width is not None:
        assert len(sig) == width
    if initval is not None:
        sig.setimmediatevalue(initval)
    return sig


class RTilePcieDevice(Device):
    def __init__(self,
                 # configuration options
                 port_num=0,
                 pcie_generation=None,
                 pcie_link_width=None,
                 pld_clk_frequency=None,
                 pf_count=1,
                 max_payload_size=128,
                 enable_extended_tag=False,

                 pf0_msi_enable=False,
                 pf0_msi_count=1,
                 pf1_msi_enable=False,
                 pf1_msi_count=1,
                 pf2_msi_enable=False,
                 pf2_msi_count=1,
                 pf3_msi_enable=False,
                 pf3_msi_count=1,
                 pf0_msix_enable=False,
                 pf0_msix_table_size=0,
                 pf0_msix_table_bir=0,
                 pf0_msix_table_offset=0x00000000,
                 pf0_msix_pba_bir=0,
                 pf0_msix_pba_offset=0x00000000,
                 pf1_msix_enable=False,
                 pf1_msix_table_size=0,
                 pf1_msix_table_bir=0,
                 pf1_msix_table_offset=0x00000000,
                 pf1_msix_pba_bir=0,
                 pf1_msix_pba_offset=0x00000000,
                 pf2_msix_enable=False,
                 pf2_msix_table_size=0,
                 pf2_msix_table_bir=0,
                 pf2_msix_table_offset=0x00000000,
                 pf2_msix_pba_bir=0,
                 pf2_msix_pba_offset=0x00000000,
                 pf3_msix_enable=False,
                 pf3_msix_table_size=0,
                 pf3_msix_table_bir=0,
                 pf3_msix_table_offset=0x00000000,
                 pf3_msix_pba_bir=0,
                 pf3_msix_pba_offset=0x00000000,

                 # signals
                 # Clock and reset
                 reset_status=None,
                 reset_status_n=None,
                 coreclkout_hip=None,
                 refclk0=None,
                 refclk1=None,
                 pin_perst_n=None,

                 # RX interface
                 rx_bus=None,
                 rx_par_err=None,

                 # TX interface
                 tx_bus=None,
                 tx_par_err=None,

                 # Power management and hard IP status interface
                 link_up=None,
                 dl_up=None,
                 surprise_down_err=None,
                 ltssm_state=None,
                 pm_state=None,
                 pm_dstate=None,
                 apps_pm_xmt_pme=None,
                 app_req_retry_en=None,

                 # Interrupt interface
                 app_int=None,
                 msi_pnd_func=None,
                 msi_pnd_byte=None,
                 msi_pnd_addr=None,

                 # Error interface
                 serr_out=None,
                 hip_enter_err_mode=None,
                 app_err_valid=None,
                 app_err_hdr=None,
                 app_err_info=None,
                 app_err_pfnum=None,

                 # Completion timeout interface
                 cpl_timeout=None,
                 cpl_timeout_avmm_clk=None,
                 cpl_timeout_avmm_address=None,
                 cpl_timeout_avmm_read=None,
                 cpl_timeout_avmm_readdata=None,
                 cpl_timeout_avmm_readdatavalid=None,
                 cpl_timeout_avmm_write=None,
                 cpl_timeout_avmm_writedata=None,
                 cpl_timeout_avmm_waitrequest=None,

                 # Configuration output
                 tl_cfg_func=None,
                 tl_cfg_add=None,
                 tl_cfg_ctl=None,
                 dl_timer_update=None,

                 # Configuration intercept interface
                 cii_req=None,
                 cii_hdr_poisoned=None,
                 cii_hdr_first_be=None,
                 cii_pfnum=None,
                 cii_wr_vf_active=None,
                 cii_vf_num=None,
                 cii_wr=None,
                 cii_addr=None,
                 cii_dout=None,
                 cii_override_en=None,
                 cii_override_din=None,
                 cii_halt=None,

                 # Hard IP reconfiguration interface
                 hip_reconfig_clk=None,
                 hip_reconfig_address=None,
                 hip_reconfig_read=None,
                 hip_reconfig_readdata=None,
                 hip_reconfig_readdatavalid=None,
                 hip_reconfig_write=None,
                 hip_reconfig_writedata=None,
                 hip_reconfig_waitrequest=None,

                 # Page request service
                 prs_event_valid=None,
                 prs_event_func=None,
                 prs_event=None,

                 # SR-IOV (VF error)
                 vf_err_ur_posted_s0=None,
                 vf_err_ur_posted_s1=None,
                 vf_err_ur_posted_s2=None,
                 vf_err_ur_posted_s3=None,
                 vf_err_pfnum_s0=None,
                 vf_err_pfnum_s1=None,
                 vf_err_pfnum_s2=None,
                 vf_err_pfnum_s3=None,
                 vf_err_ca_postedreq_s0=None,
                 vf_err_ca_postedreq_s1=None,
                 vf_err_ca_postedreq_s2=None,
                 vf_err_ca_postedreq_s3=None,
                 vf_err_vf_num_s0=None,
                 vf_err_vf_num_s1=None,
                 vf_err_vf_num_s2=None,
                 vf_err_vf_num_s3=None,
                 vf_err_poisonedwrreq_s0=None,
                 vf_err_poisonedwrreq_s1=None,
                 vf_err_poisonedwrreq_s2=None,
                 vf_err_poisonedwrreq_s3=None,
                 vf_err_poisonedcompl_s0=None,
                 vf_err_poisonedcompl_s1=None,
                 vf_err_poisonedcompl_s2=None,
                 vf_err_poisonedcompl_s3=None,
                 user_vfnonfatalmsg_pfnum=None,
                 user_vfnonfatalmsg_vfnum=None,
                 user_sent_vfnonfatalmsg=None,
                 vf_err_overflow=None,

                 # FLR
                 flr_rcvd_pf=None,
                 flr_rcvd_vf=None,
                 flr_rcvd_pf_num=None,
                 flr_rcvd_vf_num=None,
                 flr_completed_pf=None,
                 flr_completed_vf=None,
                 flr_completed_pf_num=None,
                 flr_completed_vf_num=None,

                 # VirtIO
                 virtio_pcicfg_vfaccess=None,
                 virtio_pcicfg_vfnum=None,
                 virtio_pcicfg_pfnum=None,
                 virtio_pcicfg_bar=None,
                 virtio_pcicfg_length=None,
                 virtio_pcicfg_baroffset=None,
                 virtio_pcicfg_cfgdata=None,
                 virtio_pcicfg_cfgwr=None,
                 virtio_pcicfg_cfgrd=None,
                 virtio_pcicfg_appvfnum=None,
                 virtio_pcicfg_apppfnum=None,
                 virtio_pcicfg_rdack=None,
                 virtio_pcicfg_rdbe=None,
                 virtio_pcicfg_data=None,

                 *args, **kwargs):

        super().__init__(*args, **kwargs)

        self.log.info("Intel R-tile PCIe hard IP core model")
        self.log.info("cocotbext-pcie version %s", __version__)
        self.log.info("Copyright (c) 2024 Myrfy001")
        self.log.info("https://github.com/alexforencich/cocotbext-pcie")

        self.default_function = RTilePcieFunction

        self.dw = None

        self.rx_queue = Queue()

        if port_num == 0:
            # UG lists 1444 CPLH and 2016 "512 bit" CPLD
            self.rx_buf_cplh_fc_limit = 1444
            self.rx_buf_cpld_fc_limit = 2016 * 4
        elif port_num == 1:
            self.rx_buf_cplh_fc_limit = 1144
            self.rx_buf_cpld_fc_limit = 2016 * 2
        elif port_num == 2:
            self.rx_buf_cplh_fc_limit = 572
            self.rx_buf_cpld_fc_limit = 2016
        else:
            self.rx_buf_cplh_fc_limit = 572
            self.rx_buf_cpld_fc_limit = 2016

        self.rx_buf_cplh_fc_count = 0
        self.rx_buf_cpld_fc_count = 0

        # configuration options
        self.port_num = port_num
        self.pcie_generation = pcie_generation
        self.pcie_link_width = pcie_link_width
        self.pld_clk_frequency = pld_clk_frequency
        self.pf_count = pf_count
        self.max_payload_size = max_payload_size
        self.enable_extended_tag = enable_extended_tag

        self.pf0_msi_enable = pf0_msi_enable
        self.pf0_msi_count = pf0_msi_count
        self.pf1_msi_enable = pf1_msi_enable
        self.pf1_msi_count = pf1_msi_count
        self.pf2_msi_enable = pf2_msi_enable
        self.pf2_msi_count = pf2_msi_count
        self.pf3_msi_enable = pf3_msi_enable
        self.pf3_msi_count = pf3_msi_count
        self.pf0_msix_enable = pf0_msix_enable
        self.pf0_msix_table_size = pf0_msix_table_size
        self.pf0_msix_table_bir = pf0_msix_table_bir
        self.pf0_msix_table_offset = pf0_msix_table_offset
        self.pf0_msix_pba_bir = pf0_msix_pba_bir
        self.pf0_msix_pba_offset = pf0_msix_pba_offset
        self.pf1_msix_enable = pf1_msix_enable
        self.pf1_msix_table_size = pf1_msix_table_size
        self.pf1_msix_table_bir = pf1_msix_table_bir
        self.pf1_msix_table_offset = pf1_msix_table_offset
        self.pf1_msix_pba_bir = pf1_msix_pba_bir
        self.pf1_msix_pba_offset = pf1_msix_pba_offset
        self.pf2_msix_enable = pf2_msix_enable
        self.pf2_msix_table_size = pf2_msix_table_size
        self.pf2_msix_table_bir = pf2_msix_table_bir
        self.pf2_msix_table_offset = pf2_msix_table_offset
        self.pf2_msix_pba_bir = pf2_msix_pba_bir
        self.pf2_msix_pba_offset = pf2_msix_pba_offset
        self.pf3_msix_enable = pf3_msix_enable
        self.pf3_msix_table_size = pf3_msix_table_size
        self.pf3_msix_table_bir = pf3_msix_table_bir
        self.pf3_msix_table_offset = pf3_msix_table_offset
        self.pf3_msix_pba_bir = pf3_msix_pba_bir
        self.pf3_msix_pba_offset = pf3_msix_pba_offset

        # signals

        # Clock and reset
        self.reset_status = init_signal(reset_status, 1, 0)
        self.reset_status_n = init_signal(reset_status_n, 1, 0)
        self.coreclkout_hip = init_signal(coreclkout_hip, 1, 0)
        self.refclk0 = init_signal(refclk0, 1)
        self.refclk1 = init_signal(refclk1, 1)
        self.pin_perst_n = init_signal(pin_perst_n, 1)

        # RX interface
        self.rx_source = None
        self.rx_par_err = init_signal(rx_par_err, 1, 0)

        if rx_bus is not None:
            self.rx_source = RTilePcieSource(rx_bus, self.coreclkout_hip)
            self.rx_source.queue_occupancy_limit_frames = 2
            self.rx_source.ready_latency = 27
            self.dw = self.rx_source.width

        # TX interface
        self.tx_sink = None
        self.tx_par_err = init_signal(tx_par_err, 1, 0)

        if tx_bus is not None:
            self.tx_sink = RTilePcieSink(tx_bus, self.coreclkout_hip)
            self.tx_sink.queue_occupancy_limit_frames = 2
            self.tx_sink.ready_latency = 3
            self.dw = self.tx_sink.width

        # Power management and hard IP status interface
        self.link_up = init_signal(link_up, 1, 0)
        self.dl_up = init_signal(dl_up, 1, 0)
        self.surprise_down_err = init_signal(surprise_down_err, 1, 0)
        self.ltssm_state = init_signal(ltssm_state, 6, 0)
        self.pm_state = init_signal(pm_state, 3, 0)
        self.pm_dstate = init_signal(pm_dstate, 32, 0)
        self.apps_pm_xmt_pme = init_signal(apps_pm_xmt_pme, 8)
        self.app_req_retry_en = init_signal(app_req_retry_en, 8)

        # Interrupt interface
        self.app_int = init_signal(app_int, 8)
        self.msi_pnd_func = init_signal(msi_pnd_func, 3, 0)
        self.msi_pnd_byte = init_signal(msi_pnd_byte, 8, 0)
        self.msi_pnd_addr = init_signal(msi_pnd_addr, 2, 0)

        # Error interface
        self.serr_out = init_signal(serr_out, 1, 0)
        self.hip_enter_err_mode = init_signal(hip_enter_err_mode, 1, 0)
        self.app_err_valid = init_signal(app_err_valid, 1)
        self.app_err_hdr = init_signal(app_err_hdr, 32)
        self.app_err_info = init_signal(app_err_info, 13)
        self.app_err_pfnum = init_signal(app_err_pfnum, 3)

        # Completion timeout interface
        self.cpl_timeout = init_signal(cpl_timeout, 1, 0)
        self.cpl_timeout_avmm_clk = init_signal(cpl_timeout_avmm_clk, 1)
        self.cpl_timeout_avmm_address = init_signal(
            cpl_timeout_avmm_address, 3)
        self.cpl_timeout_avmm_read = init_signal(cpl_timeout_avmm_read, 1)
        self.cpl_timeout_avmm_readdata = init_signal(
            cpl_timeout_avmm_readdata, 8, 0)
        self.cpl_timeout_avmm_readdatavalid = init_signal(
            cpl_timeout_avmm_readdatavalid, 1, 0)
        self.cpl_timeout_avmm_write = init_signal(cpl_timeout_avmm_write, 1)
        self.cpl_timeout_avmm_writedata = init_signal(
            cpl_timeout_avmm_writedata, 8)
        self.cpl_timeout_avmm_waitrequest = init_signal(
            cpl_timeout_avmm_waitrequest, 1, 0)

        # Configuration output
        self.tl_cfg_func = init_signal(tl_cfg_func, 3, 0)
        self.tl_cfg_add = init_signal(tl_cfg_add, 5, 0)
        self.tl_cfg_ctl = init_signal(tl_cfg_ctl, 16, 0)
        self.dl_timer_update = init_signal(dl_timer_update, 1, 0)

        # Configuration intercept interface
        self.cii_req = init_signal(cii_req, 1, 0)
        self.cii_hdr_poisoned = init_signal(cii_hdr_poisoned, 1, 0)
        self.cii_hdr_first_be = init_signal(cii_hdr_first_be, 4, 0)
        self.cii_pfnum = init_signal(cii_pfnum, 3, 0)
        self.cii_wr_vf_active = init_signal(cii_wr_vf_active, 1, 0)
        self.cii_vf_num = init_signal(cii_vf_num, 11, 0)
        self.cii_wr = init_signal(cii_wr, 1, 0)
        self.cii_addr = init_signal(cii_addr, 10, 0)
        self.cii_dout = init_signal(cii_dout, 32, 0)
        self.cii_override_en = init_signal(cii_override_en, 1)
        self.cii_override_din = init_signal(cii_override_din, 32)
        self.cii_halt = init_signal(cii_halt, 1)

        # Hard IP reconfiguration interface
        self.hip_reconfig_clk = init_signal(hip_reconfig_clk, 1)
        self.hip_reconfig_address = init_signal(hip_reconfig_address, 21)
        self.hip_reconfig_read = init_signal(hip_reconfig_read, 1)
        self.hip_reconfig_readdata = init_signal(hip_reconfig_readdata, 8, 0)
        self.hip_reconfig_readdatavalid = init_signal(
            hip_reconfig_readdatavalid, 1, 0)
        self.hip_reconfig_write = init_signal(hip_reconfig_write, 1)
        self.hip_reconfig_writedata = init_signal(hip_reconfig_writedata, 8)
        self.hip_reconfig_waitrequest = init_signal(
            hip_reconfig_waitrequest, 1, 0)

        # Page request service
        self.prs_event_valid = init_signal(prs_event_valid, 1)
        self.prs_event_func = init_signal(prs_event_func, 3)
        self.prs_event = init_signal(prs_event, 2)

        # SR-IOV (VF error)
        self.vf_err_ur_posted_s0 = init_signal(vf_err_ur_posted_s0, 1, 0)
        self.vf_err_ur_posted_s1 = init_signal(vf_err_ur_posted_s1, 1, 0)
        self.vf_err_ur_posted_s2 = init_signal(vf_err_ur_posted_s2, 1, 0)
        self.vf_err_ur_posted_s3 = init_signal(vf_err_ur_posted_s3, 1, 0)
        self.vf_err_pfnum_s0 = init_signal(vf_err_pfnum_s0, 3, 0)
        self.vf_err_pfnum_s1 = init_signal(vf_err_pfnum_s1, 3, 0)
        self.vf_err_pfnum_s2 = init_signal(vf_err_pfnum_s2, 3, 0)
        self.vf_err_pfnum_s3 = init_signal(vf_err_pfnum_s3, 3, 0)
        self.vf_err_ca_postedreq_s0 = init_signal(vf_err_ca_postedreq_s0, 1, 0)
        self.vf_err_ca_postedreq_s1 = init_signal(vf_err_ca_postedreq_s1, 1, 0)
        self.vf_err_ca_postedreq_s2 = init_signal(vf_err_ca_postedreq_s2, 1, 0)
        self.vf_err_ca_postedreq_s3 = init_signal(vf_err_ca_postedreq_s3, 1, 0)
        self.vf_err_vf_num_s0 = init_signal(vf_err_vf_num_s0, 11, 0)
        self.vf_err_vf_num_s1 = init_signal(vf_err_vf_num_s1, 11, 0)
        self.vf_err_vf_num_s2 = init_signal(vf_err_vf_num_s2, 11, 0)
        self.vf_err_vf_num_s3 = init_signal(vf_err_vf_num_s3, 11, 0)
        self.vf_err_poisonedwrreq_s0 = init_signal(
            vf_err_poisonedwrreq_s0, 1, 0)
        self.vf_err_poisonedwrreq_s1 = init_signal(
            vf_err_poisonedwrreq_s1, 1, 0)
        self.vf_err_poisonedwrreq_s2 = init_signal(
            vf_err_poisonedwrreq_s2, 1, 0)
        self.vf_err_poisonedwrreq_s3 = init_signal(
            vf_err_poisonedwrreq_s3, 1, 0)
        self.vf_err_poisonedcompl_s0 = init_signal(
            vf_err_poisonedcompl_s0, 1, 0)
        self.vf_err_poisonedcompl_s1 = init_signal(
            vf_err_poisonedcompl_s1, 1, 0)
        self.vf_err_poisonedcompl_s2 = init_signal(
            vf_err_poisonedcompl_s2, 1, 0)
        self.vf_err_poisonedcompl_s3 = init_signal(
            vf_err_poisonedcompl_s3, 1, 0)
        self.user_vfnonfatalmsg_pfnum = init_signal(
            user_vfnonfatalmsg_pfnum, 3)
        self.user_vfnonfatalmsg_vfnum = init_signal(
            user_vfnonfatalmsg_vfnum, 11)
        self.user_sent_vfnonfatalmsg = init_signal(user_sent_vfnonfatalmsg, 1)
        self.vf_err_overflow = init_signal(vf_err_overflow, 1, 0)

        # FLR
        self.flr_rcvd_pf = init_signal(flr_rcvd_pf, 8, 0)
        self.flr_rcvd_vf = init_signal(flr_rcvd_vf, 1, 0)
        self.flr_rcvd_pf_num = init_signal(flr_rcvd_pf_num, 3, 0)
        self.flr_rcvd_vf_num = init_signal(flr_rcvd_vf_num, 11, 0)
        self.flr_completed_pf = init_signal(flr_completed_pf, 8)
        self.flr_completed_vf = init_signal(flr_completed_vf, 1)
        self.flr_completed_pf_num = init_signal(flr_completed_pf_num, 3)
        self.flr_completed_vf_num = init_signal(flr_completed_vf_num, 11)

        # VirtIO
        self.virtio_pcicfg_vfaccess = init_signal(virtio_pcicfg_vfaccess, 1, 0)
        self.virtio_pcicfg_vfnum = init_signal(virtio_pcicfg_vfnum, 11, 0)
        self.virtio_pcicfg_pfnum = init_signal(virtio_pcicfg_pfnum, 3, 0)
        self.virtio_pcicfg_bar = init_signal(virtio_pcicfg_bar, 8, 0)
        self.virtio_pcicfg_length = init_signal(virtio_pcicfg_length, 32, 0)
        self.virtio_pcicfg_baroffset = init_signal(
            virtio_pcicfg_baroffset, 32, 0)
        self.virtio_pcicfg_cfgdata = init_signal(virtio_pcicfg_cfgdata, 32, 0)
        self.virtio_pcicfg_cfgwr = init_signal(virtio_pcicfg_cfgwr, 1, 0)
        self.virtio_pcicfg_cfgrd = init_signal(virtio_pcicfg_cfgrd, 1, 0)
        self.virtio_pcicfg_appvfnum = init_signal(virtio_pcicfg_appvfnum, 11)
        self.virtio_pcicfg_apppfnum = init_signal(virtio_pcicfg_apppfnum, 3)
        self.virtio_pcicfg_rdack = init_signal(virtio_pcicfg_rdack, 1)
        self.virtio_pcicfg_rdbe = init_signal(virtio_pcicfg_rdbe, 4)
        self.virtio_pcicfg_data = init_signal(virtio_pcicfg_data, 32)

        # validate parameters
        assert self.dw in {128, 256, 512, 1024}

        # rescale clock frequency
        if self.pld_clk_frequency is not None and self.pld_clk_frequency < 1e6:
            self.pld_clk_frequency *= 1e6

        if not self.pcie_generation or not self.pcie_link_width or not self.pld_clk_frequency:
            self.log.info(
                "Incomplete configuration specified, attempting to select reasonable options")
            # guess some reasonable values for unspecified parameters
            for config in reversed(valid_configs):
                # find configuration matching specified parameters
                if self.pcie_generation is not None and self.pcie_generation != config[0]:
                    continue
                if self.pcie_link_width is not None and self.pcie_link_width != config[1]:
                    continue
                if self.dw != config[2]:
                    continue
                if self.pld_clk_frequency is not None and self.pld_clk_frequency != config[3]:
                    continue

                if self.pcie_link_width is not None:
                    if self.port_num == 1 and config[1] > 8:
                        # port 1 only supports x4 and x8
                        continue
                    if self.port_num >= 2 and config[1] > 4:
                        # ports 2 and 3 only supports x4
                        continue

                # set the unspecified parameters
                if self.pcie_generation is None:
                    self.log.info("Setting PCIe speed to gen %d", config[0])
                    self.pcie_generation = config[0]
                if self.pcie_link_width is None:
                    self.log.info("Setting PCIe link width to x%d", config[1])
                    self.pcie_link_width = config[1]
                if self.pld_clk_frequency is None:
                    self.log.info(
                        "Setting user clock frequency to %d MHz", config[3]/1e6)
                    self.pld_clk_frequency = config[3]
                break

        self.log.info("Intel P-tile PCIe hard IP core configuration:")
        self.log.info("  PCIe speed: gen %d", self.pcie_generation)
        self.log.info("  PCIe link width: x%d", self.pcie_link_width)
        self.log.info("  PLD clock frequency: %d MHz",
                      self.pld_clk_frequency/1e6)
        self.log.info("  PF count: %d", self.pf_count)
        self.log.info("  Max payload size: %d", self.max_payload_size)
        self.log.info("  Enable extended tag: %s", self.enable_extended_tag)
        self.log.info("  P-tile port number: %d", self.port_num)
        self.log.info("  Enable PF0 MSI: %s", self.pf0_msi_enable)
        self.log.info("  PF0 MSI vector count: %d", self.pf0_msi_count)
        self.log.info("  Enable PF1 MSI: %s", self.pf1_msi_enable)
        self.log.info("  PF1 MSI vector count: %d", self.pf1_msi_count)
        self.log.info("  Enable PF2 MSI: %s", self.pf2_msi_enable)
        self.log.info("  PF2 MSI vector count: %d", self.pf2_msi_count)
        self.log.info("  Enable PF3 MSI: %s", self.pf3_msi_enable)
        self.log.info("  PF3 MSI vector count: %d", self.pf3_msi_count)
        self.log.info("  Enable PF0 MSIX: %s", self.pf0_msix_enable)
        self.log.info("  PF0 MSIX table size: %d", self.pf0_msix_table_size)
        self.log.info("  PF0 MSIX table BIR: %d", self.pf0_msix_table_bir)
        self.log.info("  PF0 MSIX table offset: 0x%08x",
                      self.pf0_msix_table_offset)
        self.log.info("  PF0 MSIX PBA BIR: %d", self.pf0_msix_pba_bir)
        self.log.info("  PF0 MSIX PBA offset: 0x%08x",
                      self.pf0_msix_pba_offset)
        self.log.info("  Enable PF1 MSIX: %s", self.pf1_msix_enable)
        self.log.info("  PF1 MSIX table size: %d", self.pf1_msix_table_size)
        self.log.info("  PF1 MSIX table BIR: %d", self.pf1_msix_table_bir)
        self.log.info("  PF1 MSIX table offset: 0x%08x",
                      self.pf1_msix_table_offset)
        self.log.info("  PF1 MSIX PBA BIR: %d", self.pf1_msix_pba_bir)
        self.log.info("  PF1 MSIX PBA offset: 0x%08x",
                      self.pf1_msix_pba_offset)
        self.log.info("  Enable PF2 MSIX: %s", self.pf2_msix_enable)
        self.log.info("  PF2 MSIX table size: %d", self.pf2_msix_table_size)
        self.log.info("  PF2 MSIX table BIR: %d", self.pf2_msix_table_bir)
        self.log.info("  PF2 MSIX table offset: 0x%08x",
                      self.pf2_msix_table_offset)
        self.log.info("  PF2 MSIX PBA BIR: %d", self.pf2_msix_pba_bir)
        self.log.info("  PF2 MSIX PBA offset: 0x%08x",
                      self.pf2_msix_pba_offset)
        self.log.info("  Enable PF3 MSIX: %s", self.pf3_msix_enable)
        self.log.info("  PF3 MSIX table size: %d", self.pf3_msix_table_size)
        self.log.info("  PF3 MSIX table BIR: %d", self.pf3_msix_table_bir)
        self.log.info("  PF3 MSIX table offset: 0x%08x",
                      self.pf3_msix_table_offset)
        self.log.info("  PF3 MSIX PBA BIR: %d", self.pf3_msix_pba_bir)
        self.log.info("  PF3 MSIX PBA offset: 0x%08x",
                      self.pf3_msix_pba_offset)

        assert self.pcie_generation in {3, 4, 5}
        assert self.pcie_link_width in {4, 8, 16}
        assert self.pld_clk_frequency in {
            175e6, 200e6, 225e6, 250e6, 350e6, 400e6, 450e6, 500e6}

        # check for valid configuration
        config_valid = False
        for config in valid_configs:
            if self.pcie_generation != config[0]:
                continue
            if self.pcie_link_width != config[1]:
                continue
            if self.dw != config[2]:
                continue
            if self.pld_clk_frequency != config[3]:
                continue

            if self.port_num == 1 and config[1] > 8:
                # port 1 only supports x4 and x8
                continue
            if self.port_num >= 2 and config[1] > 4:
                # ports 2 and 3 only supports x4
                continue

            config_valid = True
            break

        assert config_valid, "link speed/link width/clock speed/interface width setting combination not valid"

        # configure port
        self.upstream_port.max_link_speed = self.pcie_generation
        self.upstream_port.max_link_width = self.pcie_link_width

        # configure functions

        self.make_function()

        if self.pf0_msi_enable:
            self.functions[0].msi_cap.msi_multiple_message_capable = (
                self.pf0_msi_count-1).bit_length()
        else:
            self.functions[0].deregister_capability(self.functions[0].msi_cap)

        if self.pf0_msix_enable:
            self.functions[0].msix_cap.msix_table_size = self.pf0_msix_table_size
            self.functions[0].msix_cap.msix_table_bar_indicator_register = self.pf0_msix_table_bir
            self.functions[0].msix_cap.msix_table_offset = self.pf0_msix_table_offset
            self.functions[0].msix_cap.msix_pba_bar_indicator_register = self.pf0_msix_pba_bir
            self.functions[0].msix_cap.msix_pba_offset = self.pf0_msix_pba_offset
        else:
            self.functions[0].deregister_capability(self.functions[0].msix_cap)

        if self.pf_count > 1:
            self.make_function()

            if self.pf1_msi_enable:
                self.functions[1].msi_cap.msi_multiple_message_capable = (
                    self.pf1_msi_count-1).bit_length()
            else:
                self.functions[1].deregister_capability(
                    self.functions[1].msi_cap)

            if self.pf1_msix_enable:
                self.functions[1].msix_cap.msix_table_size = self.pf1_msix_table_size
                self.functions[1].msix_cap.msix_table_bar_indicator_register = self.pf1_msix_table_bir
                self.functions[1].msix_cap.msix_table_offset = self.pf1_msix_table_offset
                self.functions[1].msix_cap.msix_pba_bar_indicator_register = self.pf1_msix_pba_bir
                self.functions[1].msix_cap.msix_pba_offset = self.pf1_msix_pba_offset
            else:
                self.functions[1].deregister_capability(
                    self.functions[1].msix_cap)

        if self.pf_count > 2:
            self.make_function()

            if self.pf2_msi_enable:
                self.functions[2].msi_cap.msi_multiple_message_capable = (
                    self.pf2_msi_count-1).bit_length()
            else:
                self.functions[2].deregister_capability(
                    self.functions[2].msi_cap)

            if self.pf2_msix_enable:
                self.functions[2].msix_cap.msix_table_size = self.pf2_msix_table_size
                self.functions[2].msix_cap.msix_table_bar_indicator_register = self.pf2_msix_table_bir
                self.functions[2].msix_cap.msix_table_offset = self.pf2_msix_table_offset
                self.functions[2].msix_cap.msix_pba_bar_indicator_register = self.pf2_msix_pba_bir
                self.functions[2].msix_cap.msix_pba_offset = self.pf2_msix_pba_offset
            else:
                self.functions[2].deregister_capability(
                    self.functions[2].msix_cap)

        if self.pf_count > 3:
            self.make_function()

            if self.pf3_msi_enable:
                self.functions[3].msi_cap.msi_multiple_message_capable = (
                    self.pf3_msi_count-1).bit_length()
            else:
                self.functions[3].deregister_capability(
                    self.functions[3].msi_cap)

            if self.pf3_msix_enable:
                self.functions[3].msix_cap.msix_table_size = self.pf3_msix_table_size
                self.functions[3].msix_cap.msix_table_bar_indicator_register = self.pf3_msix_table_bir
                self.functions[3].msix_cap.msix_table_offset = self.pf3_msix_table_offset
                self.functions[3].msix_cap.msix_pba_bar_indicator_register = self.pf3_msix_pba_bir
                self.functions[3].msix_cap.msix_pba_offset = self.pf3_msix_pba_offset
            else:
                self.functions[3].deregister_capability(
                    self.functions[3].msix_cap)

        for f in self.functions:
            f.pcie_cap.max_payload_size_supported = (
                self.max_payload_size//128-1).bit_length()
            f.pcie_cap.extended_tag_supported = self.enable_extended_tag

        self.user_logic_side_fc_handler = FlowControlHandler(
            self.upstream_port.fc_state[0],
            self.coreclkout_hip,
            tx_bus,
            rx_bus
        )

        # fork coroutines

        cocotb.start_soon(self.user_logic_side_fc_handler.run())

        if self.coreclkout_hip is not None:
            cocotb.start_soon(Clock(self.coreclkout_hip, int(
                1e9/self.pld_clk_frequency), units="ns").start())

        if self.rx_source:
            cocotb.start_soon(self._run_rx_logic())
        if self.tx_sink:
            cocotb.start_soon(self._run_tx_logic())
        if self.tl_cfg_ctl:
            cocotb.start_soon(self._run_cfg_out_logic())

        cocotb.start_soon(self._run_reset())

    async def upstream_recv(self, tlp):
        self.log.debug("Got downstream TLP: %r", tlp)

        if tlp.fmt_type in {TlpType.CFG_READ_0, TlpType.CFG_WRITE_0}:
            # config type 0

            # capture address information
            self.bus_num = tlp.completer_id.bus
            self.tx_sink.set_bdf(int(tlp.completer_id))

            # pass TLP to function
            for f in self.functions:
                if f.pcie_id == tlp.completer_id:
                    await f.upstream_recv(tlp)
                    return

            tlp.release_fc()

            self.log.warning(
                "Function not found: failed to route config type 0 TLP: %r", tlp)
        elif tlp.fmt_type in {TlpType.CFG_READ_1, TlpType.CFG_WRITE_1}:
            # config type 1

            tlp.release_fc()

            self.log.warning(
                "Malformed TLP: endpoint received config type 1 TLP: %r", tlp)
        elif tlp.fmt_type in {TlpType.CPL, TlpType.CPL_DATA, TlpType.CPL_LOCKED, TlpType.CPL_LOCKED_DATA}:
            # Completion

            for f in self.functions:
                if f.pcie_id == tlp.requester_id:

                    frame = RTilePcieFrame.from_tlp(tlp)

                    frame.pfnum = tlp.requester_id.function

                    # check and track buffer occupancy
                    data_fc = tlp.get_data_credits()

                    if self.rx_buf_cplh_fc_count+1 <= self.rx_buf_cplh_fc_limit and self.rx_buf_cpld_fc_count+data_fc <= self.rx_buf_cpld_fc_limit:
                        self.rx_buf_cplh_fc_count += 1
                        self.rx_buf_cpld_fc_count += data_fc
                        await self.rx_queue.put((tlp, frame))
                    else:
                        self.log.warning("No space in RX completion buffer, dropping TLP: CPLH %d (limit %d), CPLD %d (limit %d)",
                                         self.rx_buf_cplh_fc_count, self.rx_buf_cplh_fc_limit, self.rx_buf_cpld_fc_count, self.rx_buf_cpld_fc_limit)
                        tlp.release_fc()
                    return

            tlp.release_fc()

            self.log.warning(
                "Unexpected completion: failed to route completion to function: %r", tlp)
            return  # no UR response for completion
        elif tlp.fmt_type in {TlpType.IO_READ, TlpType.IO_WRITE}:
            # IO read/write

            for f in self.functions:
                bar = f.match_bar(tlp.address, True)
                if bar:

                    frame = RTilePcieFrame.from_tlp(tlp)

                    frame.bar = 6
                    frame.pfnum = tlp.requester_id.function

                    await self.rx_queue.put((tlp, frame))

                    return

            tlp.release_fc()

            self.log.warning(
                "No BAR match: IO request did not match any BARs: %r", tlp)
        elif tlp.fmt_type in {TlpType.MEM_READ, TlpType.MEM_READ_64, TlpType.MEM_WRITE, TlpType.MEM_WRITE_64}:
            # Memory read/write

            for f in self.functions:
                bar = f.match_bar(tlp.address)
                if bar:

                    frame = RTilePcieFrame.from_tlp(tlp)

                    frame.bar = bar[0]
                    frame.pfnum = tlp.requester_id.function

                    await self.rx_queue.put((tlp, frame))

                    return

            tlp.release_fc()

            if tlp.fmt_type in {TlpType.MEM_WRITE, TlpType.MEM_WRITE_64}:
                self.log.warning(
                    "No BAR match: memory write request did not match any BARs: %r", tlp)
                return  # no UR response for write request
            else:
                self.log.warning(
                    "No BAR match: memory read request did not match any BARs: %r", tlp)
        else:
            raise Exception("TODO")

        # Unsupported request
        cpl = Tlp.create_ur_completion_for_tlp(tlp, PcieId(self.bus_num, 0, 0))
        self.log.debug("UR Completion: %r", cpl)
        await self.upstream_send(cpl)

    async def _run_reset(self):
        clock_edge_event = RisingEdge(self.coreclkout_hip)

        while True:
            await clock_edge_event
            await clock_edge_event

            if self.reset_status is not None:
                self.reset_status.value = 1
            if self.reset_status_n is not None:
                self.reset_status_n.value = 0

            if self.pin_perst_n is not None:
                if not self.pin_perst_n.value:
                    await RisingEdge(self.pin_perst_n)
                await First(FallingEdge(self.pin_perst_n), Timer(100, 'ns'))
                await First(FallingEdge(self.pin_perst_n), RisingEdge(self.coreclkout_hip))
                if not self.pin_perst_n.value:
                    continue
            else:
                await Timer(100, 'ns')
                await clock_edge_event

            if self.reset_status is not None:
                self.reset_status.value = 0
            if self.reset_status_n is not None:
                self.reset_status_n.value = 1

            if self.pin_perst_n is not None:
                await FallingEdge(self.pin_perst_n)
            else:
                return

    async def _run_rx_logic(self):
        while True:
            tlp, frame = await self.rx_queue.get()

            # block if there is congestion between rtile and user logic
            await self.user_logic_side_fc_handler.wait_until_rx_source_has_enough_credits(tlp)

            # following line will update credits for flow control between rtile and pcie link partener
            tlp.release_fc()
            # following line will update credits for flow control between rtile and user logic
            self.user_logic_side_fc_handler.consume_rx_source_credits(tlp)

            await self.rx_source.send(frame)

            # this is used for checking the overflow of hard ip core's internal cplt buffer
            self.rx_buf_cplh_fc_count = max(self.rx_buf_cplh_fc_count-1, 0)
            self.rx_buf_cpld_fc_count = max(
                self.rx_buf_cpld_fc_count-tlp.get_data_credits(), 0)

    async def _run_tx_logic(self):
        while True:
            frame = await self.tx_sink.recv()
            tlp = frame.to_tlp()
            await self.send(tlp)
            self.user_logic_side_fc_handler.release_tx_sink_credits(tlp)

    async def _run_pm_status_logic(self):
        pass

        # Power management and hard IP status interface
        # link_up
        # dl_up
        # surprise_down_err
        # ltssm_state
        # pm_state
        # pm_dstate
        # apps_pm_xmt_pme
        # app_req_retry_en

    async def _run_int_logic(self):
        pass

        # Interrupt interface
        # app_int
        # msi_pnd_func
        # msi_pnd_byte
        # msi_pnd_addr

    # Error interface
    # serr_out
    # hip_enter_err_mode
    # app_err_valid
    # app_err_hdr
    # app_err_info
    # app_err_pfnum

    # Completion timeout interface
    # cpl_timeout
    # cpl_timeout_avmm_clk
    # cpl_timeout_avmm_address
    # cpl_timeout_avmm_read
    # cpl_timeout_avmm_readdata
    # cpl_timeout_avmm_readdatavalid
    # cpl_timeout_avmm_write
    # cpl_timeout_avmm_writedata
    # cpl_timeout_avmm_waitrequest

    async def _run_cfg_out_logic(self):
        clock_edge_event = RisingEdge(self.coreclkout_hip)

        while True:
            for func in self.functions:
                self.tl_cfg_func.value = func.pcie_id.function

                self.tl_cfg_add.value = 0x00
                val = bool(func.memory_space_enable) << 15
                val |= bool(func.pcie_cap.ido_completion_enable) << 14
                val |= bool(func.parity_error_response_enable) << 13
                val |= bool(func.serr_enable) << 12
                val |= bool(func.pcie_cap.fatal_error_reporting_enable) << 11
                val |= bool(
                    func.pcie_cap.non_fatal_error_reporting_enable) << 10
                val |= bool(
                    func.pcie_cap.correctable_error_reporting_enable) << 9
                val |= bool(
                    func.pcie_cap.unsupported_request_reporting_enable) << 8
                val |= bool(func.bus_master_enable) << 7
                val |= bool(func.pcie_cap.extended_tag_field_enable) << 6
                val |= (func.pcie_cap.max_read_request_size & 0x7) << 3
                val |= (func.pcie_cap.max_payload_size & 0x7)
                self.tl_cfg_ctl.value = val
                await clock_edge_event

                self.tl_cfg_add.value = 0x01
                val = bool(func.pcie_cap.ido_request_enable) << 15
                val |= bool(func.pcie_cap.enable_no_snoop) << 14
                val |= bool(func.pcie_cap.enable_relaxed_ordering) << 13
                val |= (func.pcie_id.device & 0x1f) << 8
                val |= func.pcie_id.bus & 0xff
                self.tl_cfg_ctl.value = val
                await clock_edge_event

                self.tl_cfg_add.value = 0x02
                val = bool(func.pm_cap.no_soft_reset) << 15
                val |= bool(func.pcie_cap.read_completion_boundary) << 14
                val |= bool(func.interrupt_disable) << 13
                val |= (func.pcie_cap.interrupt_message_number & 0x1f) << 8
                val |= bool(func.pcie_cap.power_controller_control) << 4
                val |= (func.pcie_cap.attention_indicator_control & 0x3) << 2
                val |= func.pcie_cap.power_indicator_control & 0x3
                self.tl_cfg_ctl.value = val
                await clock_edge_event

                self.tl_cfg_add.value = 0x03
                # num vfs
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x04
                val = bool(func.pcie_cap.atomic_op_egress_blocking) << 14
                # ats
                val |= bool(func.pcie_cap.ari_forwarding_enable) << 7
                val |= bool(func.pcie_cap.atomic_op_requester_enable) << 6
                # tph
                # vf en
                self.tl_cfg_ctl.value = val
                await clock_edge_event

                self.tl_cfg_add.value = 0x05
                val = (func.pcie_cap.current_link_speed & 0xf) << 12
                # start vf
                self.tl_cfg_ctl.value = val
                await clock_edge_event

                self.tl_cfg_add.value = 0x06
                self.tl_cfg_ctl.value = func.msi_cap.msi_message_address & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x07
                self.tl_cfg_ctl.value = (
                    func.msi_cap.msi_message_address >> 16) & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x08
                self.tl_cfg_ctl.value = (
                    func.msi_cap.msi_message_address >> 32) & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x09
                self.tl_cfg_ctl.value = (
                    func.msi_cap.msi_message_address >> 48) & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x0A
                self.tl_cfg_ctl.value = func.msi_cap.msi_mask_bits & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x0B
                self.tl_cfg_ctl.value = (
                    func.msi_cap.msi_mask_bits >> 16) & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x0C
                val = bool(
                    func.pcie_cap.system_error_on_fatal_error_enable) << 15
                val |= bool(
                    func.pcie_cap.system_error_on_non_fatal_error_enable) << 14
                val |= bool(
                    func.pcie_cap.system_error_on_correctable_error_enable) << 13
                val |= (
                    func.aer_ext_cap.advanced_error_interrupt_message_number & 0x1f) << 8
                val |= bool(func.msi_cap.msi_extended_message_data_enable) << 7
                val |= bool(func.msix_cap.msix_function_mask) << 6
                val |= bool(func.msix_cap.msix_enable) << 5
                val |= (func.msi_cap.msi_multiple_message_enable & 0x7) << 2
                val |= bool(func.msi_cap.msi_64bit_address_capable) << 1
                val |= bool(func.msi_cap.msi_enable)
                self.tl_cfg_ctl.value = val
                await clock_edge_event

                self.tl_cfg_add.value = 0x0D
                self.tl_cfg_ctl.value = func.msi_cap.msi_message_data & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x0E
                # AER uncorrectable error mask
                val = await func.aer_ext_cap.read_register(2)
                self.tl_cfg_ctl.value = val & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x0F
                # AER uncorrectable error mask
                self.tl_cfg_ctl.value = (val >> 16) & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x10
                # AER correctable error mask
                val = await func.aer_ext_cap.read_register(5)
                self.tl_cfg_ctl.value = val & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x11
                # AER correctable error mask
                self.tl_cfg_ctl.value = (val >> 16) & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x12
                # AER uncorrectable error severity
                val = await func.aer_ext_cap.read_register(3)
                self.tl_cfg_ctl.value = val & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x13
                # AER uncorrectable error severity
                self.tl_cfg_ctl.value = (val >> 16) & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x14
                # acs
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x15
                # prs
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x16
                # prs
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x17
                # prs
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x18
                # ltr
                # pasid
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x19
                # slot control
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x1A
                # ltr max snoop lat
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x1B
                # ltr max snoop lat
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x1C
                # TC en
                val = func.pcie_cap.negotiated_link_width & 0x3f
                self.tl_cfg_ctl.value = val
                await clock_edge_event

                self.tl_cfg_add.value = 0x1D
                self.tl_cfg_ctl.value = (
                    func.msi_cap.msi_message_data >> 16) & 0xffff
                await clock_edge_event

                self.tl_cfg_add.value = 0x1E
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

                self.tl_cfg_add.value = 0x1F
                self.tl_cfg_ctl.value = 0
                await clock_edge_event

        # dl_timer_update

    # Configuration intercept interface
    # cii_req
    # cii_hdr_poisoned
    # cii_hdr_first_be
    # cii_pfnum
    # cii_wr_vf_active
    # cii_vf_num
    # cii_wr
    # cii_addr
    # cii_dout
    # cii_override_en
    # cii_override_din
    # cii_halt

    # Hard IP reconfiguration interface
    # hip_reconfig_clk
    # hip_reconfig_address
    # hip_reconfig_read
    # hip_reconfig_readdata
    # hip_reconfig_readdatavalid
    # hip_reconfig_write
    # hip_reconfig_writedata
    # hip_reconfig_waitrequest

    # Page request service
    # prs_event_valid
    # prs_event_func
    # prs_event

    # SR-IOV (VF error)
    # vf_err_ur_posted_s0
    # vf_err_ur_posted_s1
    # vf_err_ur_posted_s2
    # vf_err_ur_posted_s3
    # vf_err_pfnum_s0
    # vf_err_pfnum_s1
    # vf_err_pfnum_s2
    # vf_err_pfnum_s3
    # vf_err_ca_postedreq_s0
    # vf_err_ca_postedreq_s1
    # vf_err_ca_postedreq_s2
    # vf_err_ca_postedreq_s3
    # vf_err_vf_num_s0
    # vf_err_vf_num_s1
    # vf_err_vf_num_s2
    # vf_err_vf_num_s3
    # vf_err_poisonedwrreq_s0
    # vf_err_poisonedwrreq_s1
    # vf_err_poisonedwrreq_s2
    # vf_err_poisonedwrreq_s3
    # vf_err_poisonedcompl_s0
    # vf_err_poisonedcompl_s1
    # vf_err_poisonedcompl_s2
    # vf_err_poisonedcompl_s3
    # user_vfnonfatalmsg_pfnum
    # user_vfnonfatalmsg_vfnum
    # user_sent_vfnonfatalmsg
    # vf_err_overflow

    # FLR
    # flr_rcvd_pf
    # flr_rcvd_vf
    # flr_rcvd_pf_num
    # flr_rcvd_vf_num
    # flr_completed_pf
    # flr_completed_vf
    # flr_completed_pf_num
    # flr_completed_vf_num

    # VirtIO
    # virtio_pcicfg_vfaccess
    # virtio_pcicfg_vfnum
    # virtio_pcicfg_pfnum
    # virtio_pcicfg_bar
    # virtio_pcicfg_length
    # virtio_pcicfg_baroffset
    # virtio_pcicfg_cfgdata
    # virtio_pcicfg_cfgwr
    # virtio_pcicfg_cfgrd
    # virtio_pcicfg_appvfnum
    # virtio_pcicfg_apppfnum
    # virtio_pcicfg_rdack
    # virtio_pcicfg_rdbe
    # virtio_pcicfg_data


class CocotbSignalView:
    # note: signals[0] is the LSB in the view
    def __init__(self, signals):
        self.signals = signals

    @property
    def value(self):
        val = 0
        for signal in reversed(self.signals):
            val = val << 1
            val = val | signal.value
        return val

    @value.setter
    def value(self, val):
        for signal in self.signals:
            signal.value = val & 0x01
            val = val >> 1

    def setimmediatevalue(self, val):
        for signal in self.signals:
            signal.setimmediatevalue(val & 0x01)
            val = val >> 1

    def __len__(self):
        return len(self.signals)


class FlowControlHandler:
    def __init__(self, fc_channel_state, clk, tx_bus, rx_bus):

        self.log = logging.getLogger(
            f"cocotb.pcie.{type(self).__name__}.{id(self)}")
        self.log.name = f"cocotb.pcie.{type(self).__name__}"
        self.log.setLevel(logging.DEBUG)

        self.fc_channel_state = fc_channel_state
        self.tx_bus = tx_bus
        self.rx_bus = rx_bus

        self.ph_sink = FlowControlSinkHandlerForHeader(
            clk, tx_bus.hcrdt_update[0], CocotbSignalView([tx_bus.hcrdt_update_cnt[0], tx_bus.hcrdt_update_cnt[1]]), tx_bus.hcrdt_init[0], tx_bus.hcrdt_init_ack[0], fc_channel_state, FcType.P)
        self.nph_sink = FlowControlSinkHandlerForHeader(
            clk, tx_bus.hcrdt_update[1], CocotbSignalView([tx_bus.hcrdt_update_cnt[2], tx_bus.hcrdt_update_cnt[3]]), tx_bus.hcrdt_init[1], tx_bus.hcrdt_init_ack[1], fc_channel_state, FcType.NP)
        self.cplh_sink = FlowControlSinkHandlerForHeader(
            clk, tx_bus.hcrdt_update[2], CocotbSignalView([tx_bus.hcrdt_update_cnt[4], tx_bus.hcrdt_update_cnt[5]]), tx_bus.hcrdt_init[2], tx_bus.hcrdt_init_ack[2], fc_channel_state, FcType.CPL)
        self.pd_sink = FlowControlSinkHandlerForData(
            clk, tx_bus.dcrdt_update[0], CocotbSignalView([tx_bus.dcrdt_update_cnt[0], tx_bus.dcrdt_update_cnt[1], tx_bus.dcrdt_update_cnt[2], tx_bus.dcrdt_update_cnt[3]]), tx_bus.dcrdt_init[0], tx_bus.dcrdt_init_ack[0], fc_channel_state, FcType.P)
        self.npd_sink = FlowControlSinkHandlerForData(
            clk, tx_bus.dcrdt_update[1], CocotbSignalView([tx_bus.dcrdt_update_cnt[4], tx_bus.dcrdt_update_cnt[5], tx_bus.dcrdt_update_cnt[6], tx_bus.dcrdt_update_cnt[7]]), tx_bus.dcrdt_init[1], tx_bus.dcrdt_init_ack[1], fc_channel_state, FcType.NP)
        self.cpld_sink = FlowControlSinkHandlerForData(
            clk, tx_bus.dcrdt_update[2], CocotbSignalView([tx_bus.dcrdt_update_cnt[8], tx_bus.dcrdt_update_cnt[9], tx_bus.dcrdt_update_cnt[10], tx_bus.dcrdt_update_cnt[11]]), tx_bus.dcrdt_init[2], tx_bus.dcrdt_init_ack[2], fc_channel_state, FcType.CPL)

        self.ph_source = FlowControlSourceHandlerForHeader(
            clk, rx_bus.hcrdt_update[0], CocotbSignalView([rx_bus.hcrdt_update_cnt[0], rx_bus.hcrdt_update_cnt[1]]), rx_bus.hcrdt_init[0], rx_bus.hcrdt_init_ack[0], self, FcType.P)
        self.nph_source = FlowControlSourceHandlerForHeader(
            clk, rx_bus.hcrdt_update[1], CocotbSignalView([rx_bus.hcrdt_update_cnt[2], rx_bus.hcrdt_update_cnt[3]]), rx_bus.hcrdt_init[1], rx_bus.hcrdt_init_ack[1], self, FcType.NP)
        self.cplh_source = FlowControlSourceHandlerForHeader(
            clk, rx_bus.hcrdt_update[2], CocotbSignalView([rx_bus.hcrdt_update_cnt[4], rx_bus.hcrdt_update_cnt[5]]), rx_bus.hcrdt_init[2], rx_bus.hcrdt_init_ack[2], self, FcType.CPL)
        self.pd_source = FlowControlSourceHandlerForData(
            clk, rx_bus.dcrdt_update[0], CocotbSignalView([rx_bus.dcrdt_update_cnt[0], rx_bus.dcrdt_update_cnt[1], rx_bus.dcrdt_update_cnt[2], rx_bus.dcrdt_update_cnt[3]]), rx_bus.dcrdt_init[0], rx_bus.dcrdt_init_ack[0], self, FcType.P)
        self.npd_source = FlowControlSourceHandlerForData(
            clk, rx_bus.dcrdt_update[1], CocotbSignalView([rx_bus.dcrdt_update_cnt[4], rx_bus.dcrdt_update_cnt[5], rx_bus.dcrdt_update_cnt[6], rx_bus.dcrdt_update_cnt[7]]), rx_bus.dcrdt_init[1], rx_bus.dcrdt_init_ack[1], self, FcType.NP)
        self.cpld_source = FlowControlSourceHandlerForData(
            clk, rx_bus.dcrdt_update[2], CocotbSignalView([rx_bus.dcrdt_update_cnt[8], rx_bus.dcrdt_update_cnt[9], rx_bus.dcrdt_update_cnt[10], rx_bus.dcrdt_update_cnt[11]]), rx_bus.dcrdt_init[2], rx_bus.dcrdt_init_ack[2], self, FcType.CPL)

        self.source_credit_update_event = Event()

    def source_credit_update_callback(self):
        self.source_credit_update_event.set()

    def rx_source_has_enough_credits(self, tlp):
        data_credits_consumed = tlp.get_data_credits()
        if tlp.is_posted():
            if self.ph_source.get_available_val(
            ) >= 1 and self.pd_source.get_available_val() >= data_credits_consumed:
                return True

        elif tlp.is_nonposted():
            if self.nph_source.get_available_val(
            ) >= 1 and self.npd_source.get_available_val() >= data_credits_consumed:
                return True
        elif tlp.is_completion():
            if self.cplh_source.get_available_val(
            ) >= 1 and self.cpld_source.get_available_val() >= data_credits_consumed:
                return True
        else:
            raise Exception("should not reach here")

        return False

    async def wait_until_rx_source_has_enough_credits(self, tlp):
        while not self.rx_source_has_enough_credits(tlp):
            self.source_credit_update_event.clear()
            self.log.debug(
                "user logic side has no enough credit to store received TLP")
            await self.source_credit_update_event.wait()

    def consume_rx_source_credits(self, tlp):
        data_credits_consumed = tlp.get_data_credits()
        if tlp.is_posted():
            self.ph_source.consume_credit(1)
            self.pd_source.consume_credit(data_credits_consumed)
        elif tlp.is_nonposted():
            self.nph_source.consume_credit(1)
            self.npd_source.consume_credit(data_credits_consumed)
        else:
            self.cplh_source.consume_credit(1)
            self.cpld_source.consume_credit(data_credits_consumed)

    def release_tx_sink_credits(self, tlp):
        data_credits_consumed = tlp.get_data_credits()
        if tlp.is_posted():
            self.ph_sink.release_credit(1)
            self.pd_sink.release_credit(data_credits_consumed)
        elif tlp.is_nonposted():
            self.nph_sink.release_credit(1)
            self.npd_sink.release_credit(data_credits_consumed)
        else:
            self.cplh_sink.release_credit(1)
            self.cpld_sink.release_credit(data_credits_consumed)

    async def run(self):
        # first, wait for the RTile to communicate with PCIe link partener, and get the Credit from link partener
        await self.fc_channel_state.initialized.wait()

        self.ph_sink.set_init_value(
            self.fc_channel_state.ph.rx_initial_allocation)
        self.nph_sink.set_init_value(
            self.fc_channel_state.nph.rx_initial_allocation)
        self.cplh_sink.set_init_value(
            self.fc_channel_state.cplh.rx_initial_allocation)
        self.pd_sink.set_init_value(
            self.fc_channel_state.pd.rx_initial_allocation)
        self.npd_sink.set_init_value(
            self.fc_channel_state.npd.rx_initial_allocation)
        self.cpld_sink.set_init_value(
            self.fc_channel_state.cpld.rx_initial_allocation)


class FlowControlSourceHandlerForData:
    def __init__(self,
                 clk,
                 crdt_update,
                 crdt_update_cnt,
                 crdt_init,
                 crdt_init_ack,
                 parent_fc_handler,
                 fc_type
                 ):

        self.__dict__.setdefault('_base_field_size', 16)

        self.log = logging.getLogger(
            f"cocotb.pcie.{type(self).__name__}.{id(self)}")
        self.log.name = f"cocotb.pcie.{type(self).__name__}[{fc_type}]"
        self.log.setLevel(logging.DEBUG)

        self.clk = clk
        self.crdt_update = crdt_update
        self.crdt_update_cnt = crdt_update_cnt
        self.crdt_init = crdt_init
        self.crdt_init_ack = crdt_init_ack

        self.crdt_init_ack.setimmediatevalue(0)

        self.init_val = 0
        self.available_val = 0

        self.parent_fc_handler = parent_fc_handler
        self.fc_type = fc_type

        cocotb.start_soon(self._run())

    def get_available_val(self):
        return self.available_val

    def consume_credit(self, credit):
        if self.init_val != 0:
            self.available_val -= credit
            assert self.available_val >= 0

    async def _run(self):
        # init phase
        while True:
            if (self.crdt_init.value == 1):
                break
            await RisingEdge(self.clk)

        self.log.debug("got init request")

        self.crdt_init_ack.value = 1
        await RisingEdge(self.clk)
        while True:
            self.crdt_init_ack.value = 0
            if self.crdt_update.value == 1:
                self.init_val += self.crdt_update_cnt.value
            if self.crdt_init.value == 0:
                break
            await RisingEdge(self.clk)

        self.available_val = ((2**(self._base_field_size-1)) -
                              1) if self.init_val == 0 else self.init_val

        self.log.debug(
            f"init finished. credit advertised by user logic is: {self.init_val}")

        # normal operate stste
        while True:
            if self.crdt_update.value == 1:
                self.init_val += self.crdt_update_cnt.value
                self.parent_fc_handler.source_credit_update_callback()
            await RisingEdge(self.clk)


class FlowControlSourceHandlerForHeader(FlowControlSourceHandlerForData):
    def __init__(self, *args, **kwargs):
        self._base_field_size = 12
        super().__init__(*args, **kwargs)


class FlowControlSinkHandlerForData:
    def __init__(self,
                 clk,
                 crdt_update,
                 crdt_update_cnt,
                 crdt_init,
                 crdt_init_ack,
                 fc_channel_state,
                 fc_type
                 ):

        self.__dict__.setdefault('_base_field_size', 16)

        self.log = logging.getLogger(
            f"cocotb.pcie.{type(self).__name__}.{id(self)}")
        self.log.name = f"cocotb.pcie.{type(self).__name__}[{fc_type}]"
        self.log.setLevel(logging.DEBUG)

        self.clk = clk
        self.crdt_update = crdt_update
        self.crdt_update_cnt = crdt_update_cnt
        self.crdt_init = crdt_init
        self.crdt_init_ack = crdt_init_ack

        self.crdt_update.setimmediatevalue(0)
        self.crdt_update_cnt.setimmediatevalue(0)
        self.crdt_init.setimmediatevalue(0)

        self.init_val = None
        self.credit_to_release = None

        self.fc_channel_state = fc_channel_state
        self.fc_type = fc_type

        self.initialized = Event()

        cocotb.start_soon(self._run())

    def set_init_value(self, value):
        self.init_val = value
        self.credit_to_release = value
        self.initialized.set()

    def release_credit(self, credit):
        self.credit_to_release += credit

    async def _run(self):
        max_credit_release_per_beat = 2**(len(self.crdt_update_cnt)) - 1

        # first, wait for the RTile to communicate with PCIe link partener, and get the Credit from link partener
        await self.initialized.wait()
        self.log.debug(
            f"get credit from pcie link partener done. credit advertised by link partener is: {self.init_val}")

        # init phase
        self.crdt_init.value = 1
        while True:
            if self.crdt_init_ack.value == 1:
                break
            await RisingEdge(self.clk)

        while True:
            delta_val = min(self.credit_to_release,
                            max_credit_release_per_beat)
            self.crdt_update_cnt.value = delta_val
            self.crdt_update.value = 1
            self.credit_to_release -= delta_val
            self.log.debug(f"send credit update, val={delta_val}")
            await RisingEdge(self.clk)
            if self.credit_to_release == 0:
                self.crdt_update.value = 0
                break
        # according to user guide, must wait 2 cycle before deassert init signal
        await RisingEdge(self.clk)
        await RisingEdge(self.clk)
        self.crdt_init.value = 0
        await RisingEdge(self.clk)

        self.log.debug("init credit with user logic finished")

        # normal operate stste
        while True:
            if self.credit_to_release != 0:
                delta_val = min(self.credit_to_release,
                                max_credit_release_per_beat)
                self.crdt_update_cnt.value = delta_val
                self.crdt_update.value = 1
                self.credit_to_release -= delta_val
                self.log.debug(
                    f"send credit update in normal operation mode, val={delta_val}")
            else:
                self.crdt_update.value = 0

            await RisingEdge(self.clk)


class FlowControlSinkHandlerForHeader(FlowControlSinkHandlerForData):
    def __init__(self, *args, **kwargs):
        self._base_field_size = 12
        super().__init__(*args, **kwargs)
