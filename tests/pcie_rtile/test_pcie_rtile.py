#!/usr/bin/env python
"""

Copyright (c) 2022-2025 Alex Forencich
Copyright (c) 2026 Haru

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

import itertools
import logging
import mmap
import os
import struct

import cocotb_test.simulator
import pytest

import cocotb
from cocotb.queue import Queue
from cocotb.regression import TestFactory
from cocotb.triggers import Event, First, RisingEdge, Timer
from cocotb.handle import Immediate

from cocotbext.pcie.core.rc import RootComplex
from cocotbext.pcie.core.tlp import CplStatus, Tlp, TlpType
from cocotbext.pcie.core.utils import PcieId
from cocotbext.pcie.intel.rtile.interface import (
    RTilePcieFrame,
    RTilePcieSink,
    RTilePcieSource,
    RTileRxBus,
    RTileTxBus,
)
from cocotbext.pcie.intel.rtile.rtile_model import RTilePcieDevice, FlowControlHandler

class Credit:
    def __init__(self, value):
        self.rx_initial_allocation = value

class UserFcChannelState:
    def __init__(self, clk, ph=0, pd=0, nph=0, npd=0, cplh=0, cpld=0):
        self.clk = clk
        self.initialized = Event()
        self.ph = Credit(ph)
        self.pd = Credit(pd)
        self.nph = Credit(nph)
        self.npd = Credit(npd)
        self.cplh = Credit(cplh)
        self.cpld = Credit(cpld)
    
    async def init_credit(self, ph, pd, nph, npd, cplh, cpld):
        await RisingEdge(self.clk)
        self.ph.rx_initial_allocation = ph
        self.pd.rx_initial_allocation = pd
        self.nph.rx_initial_allocation = nph
        self.npd.rx_initial_allocation = npd
        self.cplh.rx_initial_allocation = cplh
        self.cpld.rx_initial_allocation = cpld
        self.initialized.set()


class TB:
    def __init__(self, dut, msix=False):
        self.dut = dut

        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        self.rc = RootComplex()

        dw = len(dut.tx_st_data)
        assert dw in {128, 256, 512}

        link_width = {
            128: 4,
            256: 8,
            512: 16,
        }[dw]

        # Keep the R-tile configuration intentionally conservative.  The model
        # exists, but it is less battle-tested than the P-tile path.
        self.dev = RTilePcieDevice(
            port_num=0,
            pcie_generation=3,
            pcie_link_width=link_width,
            pld_clk_frequency=250e6,
            pf_count=1,
            max_payload_size=128,
            enable_extended_tag=False,

            pf0_msi_enable=True,
            pf0_msi_count=32,
            pf1_msi_enable=False,
            pf1_msi_count=1,
            pf2_msi_enable=False,
            pf2_msi_count=1,
            pf3_msi_enable=False,
            pf3_msi_count=1,
            pf0_msix_enable=msix,
            pf0_msix_table_size=63,
            pf0_msix_table_bir=4,
            pf0_msix_table_offset=0x00000000,
            pf0_msix_pba_bir=4,
            pf0_msix_pba_offset=0x00008000,
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

            reset_status=dut.reset_status,
            reset_status_n=dut.reset_status_n,
            coreclkout_hip=dut.coreclkout_hip,
            refclk0=dut.refclk0,
            refclk1=dut.refclk1,
            pin_perst_n=dut.pin_perst_n,

            rx_bus=RTileRxBus.from_prefix(dut, "rx_st"),
            tx_bus=RTileTxBus.from_prefix(dut, "tx_st"),

            link_up=dut.link_up,
            dl_up=dut.dl_up,
            surprise_down_err=dut.surprise_down_err,
            ltssm_state=dut.ltssm_state,
            pm_state=dut.pm_state,
            pm_dstate=dut.pm_dstate,
            apps_pm_xmt_pme=dut.apps_pm_xmt_pme,
            app_req_retry_en=dut.app_req_retry_en,

            app_int=dut.app_int,
            msi_pnd_func=dut.msi_pnd_func,
            msi_pnd_byte=dut.msi_pnd_byte,
            msi_pnd_addr=dut.msi_pnd_addr,

            serr_out=dut.serr_out,
            hip_enter_err_mode=dut.hip_enter_err_mode,
            app_err_valid=dut.app_err_valid,
            app_err_hdr=dut.app_err_hdr,
            app_err_info=dut.app_err_info,
            app_err_pfnum=dut.app_err_pfnum,

            cpl_timeout=dut.cpl_timeout,
            cpl_timeout_avmm_clk=dut.cpl_timeout_avmm_clk,
            cpl_timeout_avmm_address=dut.cpl_timeout_avmm_address,
            cpl_timeout_avmm_read=dut.cpl_timeout_avmm_read,
            cpl_timeout_avmm_readdata=dut.cpl_timeout_avmm_readdata,
            cpl_timeout_avmm_readdatavalid=dut.cpl_timeout_avmm_readdatavalid,
            cpl_timeout_avmm_write=dut.cpl_timeout_avmm_write,
            cpl_timeout_avmm_writedata=dut.cpl_timeout_avmm_writedata,
            cpl_timeout_avmm_waitrequest=dut.cpl_timeout_avmm_waitrequest,

            tl_cfg_func=dut.tl_cfg_func,
            tl_cfg_add=dut.tl_cfg_add,
            tl_cfg_ctl=dut.tl_cfg_ctl,
            dl_timer_update=dut.dl_timer_update,

            cii_req=dut.cii_req,
            cii_hdr_poisoned=dut.cii_hdr_poisoned,
            cii_hdr_first_be=dut.cii_hdr_first_be,
            cii_pfnum=dut.cii_pfnum,
            cii_wr_vf_active=dut.cii_wr_vf_active,
            cii_vf_num=dut.cii_vf_num,
            cii_wr=dut.cii_wr,
            cii_addr=dut.cii_addr,
            cii_dout=dut.cii_dout,
            cii_override_en=dut.cii_override_en,
            cii_override_din=dut.cii_override_din,
            cii_halt=dut.cii_halt,

            hip_reconfig_clk=dut.hip_reconfig_clk,
            hip_reconfig_address=dut.hip_reconfig_address,
            hip_reconfig_read=dut.hip_reconfig_read,
            hip_reconfig_readdata=dut.hip_reconfig_readdata,
            hip_reconfig_readdatavalid=dut.hip_reconfig_readdatavalid,
            hip_reconfig_write=dut.hip_reconfig_write,
            hip_reconfig_writedata=dut.hip_reconfig_writedata,
            hip_reconfig_waitrequest=dut.hip_reconfig_waitrequest,

            prs_event_valid=dut.prs_event_valid,
            prs_event_func=dut.prs_event_func,
            prs_event=dut.prs_event,

            vf_err_ur_posted_s0=dut.vf_err_ur_posted_s0,
            vf_err_ur_posted_s1=dut.vf_err_ur_posted_s1,
            vf_err_ur_posted_s2=dut.vf_err_ur_posted_s2,
            vf_err_ur_posted_s3=dut.vf_err_ur_posted_s3,
            vf_err_pfnum_s0=dut.vf_err_pfnum_s0,
            vf_err_pfnum_s1=dut.vf_err_pfnum_s1,
            vf_err_pfnum_s2=dut.vf_err_pfnum_s2,
            vf_err_pfnum_s3=dut.vf_err_pfnum_s3,
            vf_err_ca_postedreq_s0=dut.vf_err_ca_postedreq_s0,
            vf_err_ca_postedreq_s1=dut.vf_err_ca_postedreq_s1,
            vf_err_ca_postedreq_s2=dut.vf_err_ca_postedreq_s2,
            vf_err_ca_postedreq_s3=dut.vf_err_ca_postedreq_s3,
            vf_err_vf_num_s0=dut.vf_err_vf_num_s0,
            vf_err_vf_num_s1=dut.vf_err_vf_num_s1,
            vf_err_vf_num_s2=dut.vf_err_vf_num_s2,
            vf_err_vf_num_s3=dut.vf_err_vf_num_s3,
            vf_err_poisonedwrreq_s0=dut.vf_err_poisonedwrreq_s0,
            vf_err_poisonedwrreq_s1=dut.vf_err_poisonedwrreq_s1,
            vf_err_poisonedwrreq_s2=dut.vf_err_poisonedwrreq_s2,
            vf_err_poisonedwrreq_s3=dut.vf_err_poisonedwrreq_s3,
            vf_err_poisonedcompl_s0=dut.vf_err_poisonedcompl_s0,
            vf_err_poisonedcompl_s1=dut.vf_err_poisonedcompl_s1,
            vf_err_poisonedcompl_s2=dut.vf_err_poisonedcompl_s2,
            vf_err_poisonedcompl_s3=dut.vf_err_poisonedcompl_s3,
            user_vfnonfatalmsg_pfnum=dut.user_vfnonfatalmsg_pfnum,
            user_vfnonfatalmsg_vfnum=dut.user_vfnonfatalmsg_vfnum,
            user_sent_vfnonfatalmsg=dut.user_sent_vfnonfatalmsg,
            vf_err_overflow=dut.vf_err_overflow,

            flr_rcvd_pf=dut.flr_rcvd_pf,
            flr_rcvd_vf=dut.flr_rcvd_vf,
            flr_rcvd_pf_num=dut.flr_rcvd_pf_num,
            flr_rcvd_vf_num=dut.flr_rcvd_vf_num,
            flr_completed_pf=dut.flr_completed_pf,
            flr_completed_vf=dut.flr_completed_vf,
            flr_completed_pf_num=dut.flr_completed_pf_num,
            flr_completed_vf_num=dut.flr_completed_vf_num,

            virtio_pcicfg_vfaccess=dut.virtio_pcicfg_vfaccess,
            virtio_pcicfg_vfnum=dut.virtio_pcicfg_vfnum,
            virtio_pcicfg_pfnum=dut.virtio_pcicfg_pfnum,
            virtio_pcicfg_bar=dut.virtio_pcicfg_bar,
            virtio_pcicfg_length=dut.virtio_pcicfg_length,
            virtio_pcicfg_baroffset=dut.virtio_pcicfg_baroffset,
            virtio_pcicfg_cfgdata=dut.virtio_pcicfg_cfgdata,
            virtio_pcicfg_cfgwr=dut.virtio_pcicfg_cfgwr,
            virtio_pcicfg_cfgrd=dut.virtio_pcicfg_cfgrd,
            virtio_pcicfg_appvfnum=dut.virtio_pcicfg_appvfnum,
            virtio_pcicfg_apppfnum=dut.virtio_pcicfg_apppfnum,
            virtio_pcicfg_rdack=dut.virtio_pcicfg_rdack,
            virtio_pcicfg_rdbe=dut.virtio_pcicfg_rdbe,
            virtio_pcicfg_data=dut.virtio_pcicfg_data,
        )

        self.dev.log.setLevel(logging.DEBUG)

        dut.refclk0.set(Immediate(0))
        dut.refclk1.set(Immediate(0))
        dut.pin_perst_n.value = 1
        dut.apps_pm_xmt_pme.set(Immediate(0))
        dut.app_req_retry_en.set(Immediate(0))
        dut.app_int.set(Immediate(0))
        dut.msi_pnd_func.set(Immediate(0))
        dut.msi_pnd_byte.set(Immediate(0))
        dut.msi_pnd_addr.set(Immediate(0))
        dut.app_err_valid.set(Immediate(0))
        dut.app_err_hdr.set(Immediate(0))
        dut.app_err_info.set(Immediate(0))
        dut.app_err_pfnum.set(Immediate(0))
        dut.cpl_timeout.set(Immediate(0))
        dut.cpl_timeout_avmm_clk.set(Immediate(0))
        dut.cpl_timeout_avmm_address.set(Immediate(0))
        dut.cpl_timeout_avmm_read.set(Immediate(0))
        dut.cpl_timeout_avmm_write.set(Immediate(0))
        dut.cpl_timeout_avmm_writedata.set(Immediate(0))
        dut.cii_override_en.set(Immediate(0))
        dut.cii_override_din.set(Immediate(0))
        dut.cii_halt.set(Immediate(0))
        dut.hip_reconfig_clk.set(Immediate(0))
        dut.hip_reconfig_address.set(Immediate(0))
        dut.hip_reconfig_read.set(Immediate(0))
        dut.hip_reconfig_write.set(Immediate(0))
        dut.hip_reconfig_writedata.set(Immediate(0))
        dut.prs_event_valid.set(Immediate(0))
        dut.prs_event_func.set(Immediate(0))
        dut.prs_event.set(Immediate(0))
        dut.user_vfnonfatalmsg_pfnum.set(Immediate(0))
        dut.user_vfnonfatalmsg_vfnum.set(Immediate(0))
        dut.user_sent_vfnonfatalmsg.set(Immediate(0))
        dut.flr_completed_pf.set(Immediate(0))
        dut.flr_completed_vf.set(Immediate(0))
        dut.flr_completed_pf_num.set(Immediate(0))
        dut.flr_completed_vf_num.set(Immediate(0))
        dut.virtio_pcicfg_appvfnum.set(Immediate(0))
        dut.virtio_pcicfg_apppfnum.set(Immediate(0))
        dut.virtio_pcicfg_rdack.set(Immediate(0))
        dut.virtio_pcicfg_rdbe.set(Immediate(0))
        dut.virtio_pcicfg_data.set(Immediate(0))
        dut.tx_ehp_deallocate_empty.set(Immediate(0))

        self.rc.make_port().connect(self.dev)
        rtile_tx_bus = RTileTxBus.from_prefix(dut, "tx_st")
        rtile_rx_bus = RTileRxBus.from_prefix(dut, "rx_st")
        self.tx_source = RTilePcieSource(rtile_tx_bus, dut.coreclkout_hip)
        self.tx_source.ready_latency = 3
        self.rx_sink = RTilePcieSink(rtile_rx_bus, dut.coreclkout_hip)
        # TODO: set bdf???????
        self.rx_sink.set_bdf(256) # set bus 1, device 0, function 0
        self.rx_sink.ready_latency = 27

        self.fc_channel_state = UserFcChannelState(dut.coreclkout_hip)
        self.fc = FlowControlHandler(self.fc_channel_state, dut.coreclkout_hip, rtile_rx_bus, rtile_tx_bus)

        self.regions = [None] * 6
        self.regions[0] = mmap.mmap(-1, 1024 * 1024)
        self.regions[1] = mmap.mmap(-1, 1024 * 1024)
        self.regions[3] = mmap.mmap(-1, 1024)
        self.regions[4] = mmap.mmap(-1, 1024 * 64)

        self.current_tag = 0
        self.tag_count = 256

        self.tag_active = [False] * 256
        self.tag_release = Event()

        self.rx_cpl_queues = [Queue() for _ in range(256)]
        self.rx_cpl_sync = [Event() for _ in range(256)]

        self.dev_bus_num = 0
        self.dev_device_num = 0
        self.dev_max_payload = 0
        self.dev_max_read_req = 0
        self.dev_msi_enable = 0
        self.dev_msi_multi_msg_enable = 0
        self.dev_msi_address = 0
        self.dev_msi_data = 0
        self.dev_msi_mask = 0
        self.dev_msix_enable = 0
        self.dev_msix_function_mask = 0

        self.dev.functions[0].configure_bar(0, len(self.regions[0]))
        self.dev.functions[0].configure_bar(1, len(self.regions[1]), True, True)
        self.dev.functions[0].configure_bar(3, len(self.regions[3]), False, False, True)
        self.dev.functions[0].configure_bar(4, len(self.regions[4]))

        cocotb.start_soon(self.fc_channel_state.init_credit(64, 1024, 64, 64, 0, 0))
        cocotb.start_soon(self.fc.run())
        cocotb.start_soon(self._run_rx_tlp())
        cocotb.start_soon(self._run_cfg())

    def set_idle_generator(self, generator=None):
        if generator:
            self.dev.rx_source.set_pause_generator(generator())

    def set_backpressure_generator(self, generator=None):
        if generator:
            self.dev.tx_sink.set_pause_generator(generator())

    async def recv_cpl(self, tag, timeout=0, timeout_unit="ns"):
        queue = self.rx_cpl_queues[tag]
        sync = self.rx_cpl_sync[tag]

        if not queue.empty():
            return queue.get_nowait()

        sync.clear()
        if timeout:
            await First(sync.wait(), Timer(timeout, timeout_unit))
        else:
            await sync.wait()

        if not queue.empty():
            return queue.get_nowait()

        return None

    async def alloc_tag(self):
        tag_count = min(256 if self.dev.functions[0].pcie_cap.extended_tag_field_enable else 32, self.tag_count)

        while True:
            tag = self.current_tag
            for _ in range(tag_count):
                tag = (tag + 1) % tag_count
                if not self.tag_active[tag]:
                    self.tag_active[tag] = True
                    self.current_tag = tag
                    return tag

            self.tag_release.clear()
            await self.tag_release.wait()

    def release_tag(self, tag):
        assert self.tag_active[tag]
        self.tag_active[tag] = False
        self.tag_release.set()

    async def perform_posted_operation(self, req):
        # await self.tx_source.send(RTilePcieFrame.from_tlp(req))
        await self.fc.wait_until_rx_source_has_enough_credits(req)

        await self.tx_source.send(RTilePcieFrame.from_tlp(req))

        self.fc.consume_rx_source_credits(req)

    async def perform_nonposted_operation(self, req, timeout=0, timeout_unit="ns"):
        completions = []

        req.tag = await self.alloc_tag()

        # await self.tx_source.send(RTilePcieFrame.from_tlp(req))
        await self.fc.wait_until_rx_source_has_enough_credits(req)

        await self.tx_source.send(RTilePcieFrame.from_tlp(req))

        self.fc.consume_rx_source_credits(req)

        while True:
            cpl = await self.recv_cpl(req.tag, timeout, timeout_unit)

            if not cpl:
                break

            completions.append(cpl)

            if cpl.status != CplStatus.SC:
                break
            if req.fmt_type in {TlpType.MEM_READ, TlpType.MEM_READ_64}:
                if cpl.byte_count <= cpl.length * 4 - (cpl.lower_address & 0x3):
                    break
                if cpl.fmt_type in {TlpType.CPL, TlpType.CPL_LOCKED}:
                    break
            else:
                break

        self.release_tag(req.tag)

        return completions

    async def dma_io_write(self, addr, data, timeout=0, timeout_unit="ns"):
        n = 0

        zero_len = len(data) == 0
        if zero_len:
            data = b"\x00"

        op_list = []

        while n < len(data):
            req = Tlp()
            req.fmt_type = TlpType.IO_WRITE
            req.requester_id = PcieId(self.dev_bus_num, self.dev_device_num, 0)

            first_pad = addr % 4
            byte_length = min(len(data) - n, 4 - first_pad)
            req.set_addr_be_data(addr, data[n:n + byte_length])

            if zero_len:
                req.first_be = 0

            op_list.append(cocotb.start_soon(self.perform_nonposted_operation(req, timeout, timeout_unit)))

            n += byte_length
            addr += byte_length

        for op in op_list:
            cpl_list = await op

            if not cpl_list:
                raise Exception("Timeout")
            if cpl_list[0].status != CplStatus.SC:
                raise Exception("Unsuccessful completion")

    async def dma_io_read(self, addr, length, timeout=0, timeout_unit="ns"):
        data = bytearray()
        n = 0

        zero_len = length <= 0
        if zero_len:
            length = 1

        op_list = []

        while n < length:
            req = Tlp()
            req.fmt_type = TlpType.IO_READ
            req.requester_id = PcieId(self.dev_bus_num, self.dev_device_num, 0)

            first_pad = addr % 4
            byte_length = min(length - n, 4 - first_pad)
            req.set_addr_be(addr, byte_length)

            if zero_len:
                req.first_be = 0

            op_list.append((first_pad, cocotb.start_soon(self.perform_nonposted_operation(req, timeout, timeout_unit))))

            n += byte_length
            addr += byte_length

        for first_pad, op in op_list:
            cpl_list = await op

            if not cpl_list:
                raise Exception("Timeout")
            cpl = cpl_list[0]
            if cpl.status != CplStatus.SC:
                raise Exception("Unsuccessful completion")

            assert cpl.length == 1
            d = cpl.get_data()

            data.extend(d[first_pad:])

        if zero_len:
            return b""

        return bytes(data[:length])

    async def dma_mem_write(self, addr, data, timeout=0, timeout_unit="ns"):
        n = 0

        zero_len = len(data) == 0
        if zero_len:
            data = b"\x00"

        while n < len(data):
            req = Tlp()
            if addr > 0xFFFFFFFF:
                req.fmt_type = TlpType.MEM_WRITE_64
            else:
                req.fmt_type = TlpType.MEM_WRITE
            req.requester_id = PcieId(self.dev_bus_num, self.dev_device_num, 0)

            first_pad = addr % 4
            byte_length = len(data) - n
            byte_length = min(byte_length, (128 << self.dev_max_payload) - first_pad)
            byte_length = min(byte_length, 0x1000 - (addr & 0xFFF))
            req.set_addr_be_data(addr, data[n:n + byte_length])

            if zero_len:
                req.first_be = 0

            await self.perform_posted_operation(req)

            n += byte_length
            addr += byte_length

    async def dma_mem_read(self, addr, length, timeout=0, timeout_unit="ns"):
        data = bytearray()
        n = 0

        zero_len = length <= 0
        if zero_len:
            length = 1

        op_list = []

        while n < length:
            req = Tlp()
            if addr > 0xFFFFFFFF:
                req.fmt_type = TlpType.MEM_READ_64
            else:
                req.fmt_type = TlpType.MEM_READ
            req.requester_id = PcieId(self.dev_bus_num, self.dev_device_num, 0)

            first_pad = addr % 4
            byte_length = length - n
            if byte_length > (128 << self.dev_max_read_req) - first_pad:
                byte_length = min(byte_length, (128 << self.dev_max_read_req) - (addr & 0x7F))
            byte_length = min(byte_length, 0x1000 - (addr & 0xFFF))
            req.set_addr_be(addr, byte_length)

            if zero_len:
                req.first_be = 0

            op_list.append((byte_length, cocotb.start_soon(self.perform_nonposted_operation(req, timeout, timeout_unit))))

            n += byte_length
            addr += byte_length

        for byte_length, op in op_list:
            cpl_list = await op

            m = 0

            while m < byte_length:
                if not cpl_list:
                    raise Exception("Timeout")

                cpl = cpl_list.pop(0)

                if cpl.status != CplStatus.SC:
                    raise Exception("Unsuccessful completion")

                assert cpl.byte_count + 3 + (cpl.lower_address & 3) >= cpl.length * 4
                assert cpl.byte_count == max(byte_length - m, 1)

                d = cpl.get_data()

                offset = cpl.lower_address & 3
                data.extend(d[offset:offset + cpl.byte_count])

                m += len(d) - offset

        if zero_len:
            return b""

        return bytes(data[:length])

    async def _run_rx_tlp(self):
        while True:
            frame = await self.rx_sink.recv()

            tlp = frame.to_tlp()
            self.fc.release_tx_sink_credits(tlp)

            self.log.debug("RX TLP: %s", repr(tlp))

            if tlp.fmt_type in {TlpType.CPL, TlpType.CPL_DATA, TlpType.CPL_LOCKED, TlpType.CPL_LOCKED_DATA}:
                self.log.info("Completion")

                self.rx_cpl_queues[tlp.tag].put_nowait(tlp)
                self.rx_cpl_sync[tlp.tag].set()
                continue

            elif tlp.fmt_type == TlpType.IO_READ:
                self.log.info("IO read")

                cpl = Tlp.create_completion_data_for_tlp(tlp, PcieId(self.dev_bus_num, 0, 0))

                region = 3 if frame.bar == 6 else frame.bar
                addr = tlp.address % len(self.regions[region])
                offset = 0
                start_offset = None
                mask = tlp.first_be

                data = bytearray(4)

                for k in range(4):
                    if mask & (1 << k):
                        if start_offset is None:
                            start_offset = offset
                    else:
                        if start_offset is not None and offset != start_offset:
                            data[start_offset:offset] = self.regions[region][addr + start_offset:addr + offset]
                        start_offset = None

                    offset += 1

                if start_offset is not None and offset != start_offset:
                    data[start_offset:offset] = self.regions[region][addr + start_offset:addr + offset]

                cpl.set_data(data)
                cpl.byte_count = 4
                cpl.length = 1

                self.log.debug("Completion: %s", repr(cpl))
                # await self.tx_source.send(RTilePcieFrame.from_tlp(cpl))
                await self.fc.wait_until_rx_source_has_enough_credits(cpl)

                await self.tx_source.send(RTilePcieFrame.from_tlp(cpl))

                self.fc.consume_rx_source_credits(cpl)

            elif tlp.fmt_type == TlpType.IO_WRITE:
                self.log.info("IO write")

                cpl = Tlp.create_completion_for_tlp(tlp, PcieId(self.dev_bus_num, 0, 0))

                region = 3 if frame.bar == 6 else frame.bar
                addr = tlp.address % len(self.regions[region])
                offset = 0
                start_offset = None
                mask = tlp.first_be

                data = tlp.get_data()

                for k in range(4):
                    if mask & (1 << k):
                        if start_offset is None:
                            start_offset = offset
                    else:
                        if start_offset is not None and offset != start_offset:
                            self.regions[region][addr + start_offset:addr + offset] = data[start_offset:offset]
                        start_offset = None

                    offset += 1

                if start_offset is not None and offset != start_offset:
                    self.regions[region][addr + start_offset:addr + offset] = data[start_offset:offset]

                self.log.debug("Completion: %s", repr(cpl))
                # await self.tx_source.send(RTilePcieFrame.from_tlp(cpl))
                await self.fc.wait_until_rx_source_has_enough_credits(cpl)

                await self.tx_source.send(RTilePcieFrame.from_tlp(cpl))

                self.fc.consume_rx_source_credits(cpl)

            elif tlp.fmt_type in {TlpType.MEM_READ, TlpType.MEM_READ_64}:
                self.log.info("Memory read")

                region = frame.bar
                addr = tlp.address % len(self.regions[region])
                length = tlp.length

                data = self.regions[region][addr:addr + length * 4]

                m = 0
                n = 0
                addr = tlp.address + tlp.get_first_be_offset()
                dw_length = tlp.length
                byte_length = tlp.get_be_byte_count()

                while m < dw_length:
                    cpl = Tlp.create_completion_data_for_tlp(tlp, PcieId(0, 0, 0))

                    cpl_dw_length = dw_length - m
                    cpl_byte_length = byte_length - n
                    cpl.byte_count = cpl_byte_length
                    if cpl_dw_length > 32 << self.dev_max_payload:
                        cpl_dw_length = 32 << self.dev_max_payload
                        cpl_dw_length -= (addr & 0x7C) >> 2

                    cpl.lower_address = addr & 0x7F

                    cpl.set_data(data[m * 4:(m + cpl_dw_length) * 4])

                    self.log.debug("Completion: %s", repr(cpl))
                    # await self.tx_source.send(RTilePcieFrame.from_tlp(cpl))
                    await self.fc.wait_until_rx_source_has_enough_credits(cpl)

                    await self.tx_source.send(RTilePcieFrame.from_tlp(cpl))

                    self.fc.consume_rx_source_credits(cpl)

                    m += cpl_dw_length
                    n += cpl_dw_length * 4 - (addr & 3)
                    addr += cpl_dw_length * 4 - (addr & 3)

            elif tlp.fmt_type in {TlpType.MEM_WRITE, TlpType.MEM_WRITE_64}:
                self.log.info("Memory write")

                region = frame.bar
                addr = tlp.address % len(self.regions[region])
                offset = 0
                start_offset = None
                mask = tlp.first_be
                length = tlp.length

                data = tlp.get_data()

                for k in range(4):
                    if mask & (1 << k):
                        if start_offset is None:
                            start_offset = offset
                    else:
                        if start_offset is not None and offset != start_offset:
                            self.regions[region][addr + start_offset:addr + offset] = data[start_offset:offset]
                        start_offset = None

                    offset += 1

                if length > 2:
                    if start_offset is None:
                        start_offset = offset
                    offset += (length - 2) * 4

                if length > 1:
                    mask = tlp.last_be

                    for k in range(4):
                        if mask & (1 << k):
                            if start_offset is None:
                                start_offset = offset
                        else:
                            if start_offset is not None and offset != start_offset:
                                self.regions[region][addr + start_offset:addr + offset] = data[start_offset:offset]
                            start_offset = None

                        offset += 1

                if start_offset is not None and offset != start_offset:
                    self.regions[region][addr + start_offset:addr + offset] = data[start_offset:offset]

    async def _run_cfg(self):
        while True:
            await RisingEdge(self.dut.coreclkout_hip)

            try:
                func = int(self.dut.tl_cfg_func.value)
                addr = int(self.dut.tl_cfg_add.value)
                ctl = int(self.dut.tl_cfg_ctl.value)
            except ValueError:
                continue

            if func == 0:
                if addr == 0x00:
                    self.dev_max_payload = ctl & 0x7
                    self.dev_max_read_req = (ctl >> 3) & 0x7
                elif addr == 0x01:
                    self.dev_bus_num = ctl & 0xFF
                    self.dev_device_num = (ctl >> 8) & 0x1F
                elif addr == 0x06:
                    self.dev_msi_address = (self.dev_msi_address & ~(0xFFFF << 0)) | (ctl << 0)
                elif addr == 0x07:
                    self.dev_msi_address = (self.dev_msi_address & ~(0xFFFF << 16)) | (ctl << 16)
                elif addr == 0x08:
                    self.dev_msi_address = (self.dev_msi_address & ~(0xFFFF << 32)) | (ctl << 32)
                elif addr == 0x09:
                    self.dev_msi_address = (self.dev_msi_address & ~(0xFFFF << 48)) | (ctl << 48)
                elif addr == 0x0A:
                    self.dev_msi_mask = (self.dev_msi_mask & ~(0xFFFF << 0)) | (ctl << 0)
                elif addr == 0x0B:
                    self.dev_msi_mask = (self.dev_msi_mask & ~(0xFFFF << 16)) | (ctl << 16)
                elif addr == 0x0C:
                    self.dev_msi_enable = ctl & 1
                    self.dev_msi_multi_msg_enable = (ctl >> 2) & 0x7
                    self.dev_msix_enable = (ctl >> 5) & 1
                    self.dev_msix_function_mask = (ctl >> 6) & 1
                elif addr == 0x0D:
                    self.dev_msi_data = (self.dev_msi_data & ~(0xFFFF << 0)) | (ctl << 0)
                elif addr == 0x1D:
                    self.dev_msi_data = (self.dev_msi_data & ~(0xFFFF << 16)) | (ctl << 16)


async def run_test_mem(dut, idle_inserter=None, backpressure_inserter=None):
    tb = TB(dut)

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await RisingEdge(dut.reset_status_n)
    await Timer(100, "ns")

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()

    dev_bar0 = dev.bar_window[0]
    dev_bar1 = dev.bar_window[1]
    dev_bar3 = dev.bar_window[3]

    for length in list(range(0, 8)):
        for offset in list(range(8)):
            tb.log.info("IO operation length: %d offset: %d", length, offset)
            test_data = bytearray([x % 256 for x in range(length)])

            await dev_bar3.write(offset, test_data, timeout=5000)
            assert tb.regions[3][offset:offset + length] == test_data

            assert await dev_bar3.read(offset, length, timeout=5000) == test_data

    for length in list(range(0, 32)) + [1024]:
        for offset in list(range(8)) + list(range(4096 - 8, 4096)):
            tb.log.info("Memory operation (32-bit BAR) length: %d offset: %d", length, offset)
            test_data = bytearray([x % 256 for x in range(length)])

            await dev_bar0.write(offset, test_data, timeout=100)
            await dev_bar0.read(offset, 0, timeout=5000)
            assert tb.regions[0][offset:offset + length] == test_data

            assert await dev_bar0.read(offset, length, timeout=5000) == test_data

    for length in list(range(0, 32)) + [1024]:
        for offset in list(range(8)) + list(range(4096 - 8, 4096)):
            tb.log.info("Memory operation (64-bit BAR) length: %d offset: %d", length, offset)
            test_data = bytearray([x % 256 for x in range(length)])

            await dev_bar1.write(offset, test_data, timeout=100)
            await dev_bar1.read(offset, 0, timeout=5000)
            assert tb.regions[1][offset:offset + length] == test_data

            assert await dev_bar1.read(offset, length, timeout=5000) == test_data

    await RisingEdge(dut.coreclkout_hip)
    await RisingEdge(dut.coreclkout_hip)


async def run_test_dma(dut, idle_inserter=None, backpressure_inserter=None):
    tb = TB(dut)

    mem = tb.rc.mem_pool.alloc_region(16 * 1024 * 1024)
    mem_base = mem.get_absolute_address(0)

    io = tb.rc.io_pool.alloc_region(1024)
    io_base = io.get_absolute_address(0)

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await RisingEdge(dut.reset_status_n)
    await Timer(100, "ns")

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()
    await dev.set_master()

    for length in list(range(0, 32)) + [1024]:
        for offset in list(range(8)) + list(range(4096 - 8, 4096)):
            tb.log.info("Memory operation (DMA) length: %d offset: %d", length, offset)
            addr = mem_base + offset
            test_data = bytearray([x % 256 for x in range(length)])

            await tb.dma_mem_write(addr, test_data, 5000, "ns")
            await tb.dma_mem_read(addr, 0, 5000, "ns")
            assert mem[offset:offset + length] == test_data

            assert await tb.dma_mem_read(addr, length, 5000, "ns") == test_data

    for length in list(range(0, 8)):
        for offset in list(range(8)):
            tb.log.info("IO operation (DMA) length: %d offset: %d", length, offset)
            addr = io_base + offset
            test_data = bytearray([x % 256 for x in range(length)])

            await tb.dma_io_write(addr, test_data, 5000, "ns")
            assert io[offset:offset + length] == test_data

            assert await tb.dma_io_read(addr, length, 5000, "ns") == test_data

    await RisingEdge(dut.coreclkout_hip)
    await RisingEdge(dut.coreclkout_hip)


async def run_test_msi(dut, idle_inserter=None, backpressure_inserter=None):
    tb = TB(dut)

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await RisingEdge(dut.reset_status_n)
    await Timer(100, "ns")

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()
    await dev.set_master()
    await dev.alloc_irq_vectors(32, 32)

    await Timer(100, "ns")
    assert tb.dev_msi_enable

    for k in range(32):
        tb.log.info("Send MSI %d", k)

        data = tb.dev_msi_data & ~(2 ** tb.dev_msi_multi_msg_enable - 1) | k
        await tb.dma_mem_write(tb.dev_msi_address, struct.pack("<L", data))

        event = dev.msi_vectors[k].event
        event.clear()
        await event.wait()

    await RisingEdge(dut.coreclkout_hip)
    await RisingEdge(dut.coreclkout_hip)


async def run_test_msix(dut, idle_inserter=None, backpressure_inserter=None):
    tb = TB(dut, msix=True)

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    await RisingEdge(dut.reset_status_n)
    await Timer(100, "ns")

    await tb.rc.enumerate()

    dev = tb.rc.find_device(tb.dev.functions[0].pcie_id)
    await dev.enable_device()
    await dev.set_master()
    await dev.alloc_irq_vectors(64, 64)

    await Timer(100, "ns")
    assert tb.dev_msix_enable

    for k in range(64):
        tb.log.info("Send MSI %d", k)

        addr = int.from_bytes(tb.regions[4][16 * k + 0:16 * k + 8], "little")
        data = int.from_bytes(tb.regions[4][16 * k + 8:16 * k + 12], "little")

        await tb.dma_mem_write(addr, data.to_bytes(4, "little"), 5000, "ns")

        event = dev.msi_vectors[k].event
        event.clear()
        await event.wait()

    await RisingEdge(dut.coreclkout_hip)
    await RisingEdge(dut.coreclkout_hip)


def cycle_pause():
    return itertools.cycle([1, 1, 1, 0])


if getattr(cocotb, "top", None) is not None:
    for test in [
        run_test_mem,
        run_test_dma,
        run_test_msi,
        run_test_msix,
    ]:
        factory = TestFactory(test)
        factory.add_option(("idle_inserter", "backpressure_inserter"), [(None, None), (cycle_pause, cycle_pause)])
        factory.generate_tests()


tests_dir = os.path.dirname(__file__)


@pytest.mark.parametrize("data_width", [128, 256, 512])
def test_pcie_rtile(request, data_width):
    dut = "test_pcie_rtile"
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = dut

    verilog_sources = [
        os.path.join(tests_dir, f"{dut}.v"),
    ]

    parameters = {}

    parameters["SEG_COUNT"] = 2 if data_width == 512 else 1
    parameters["SEG_DATA_WIDTH"] = data_width // parameters["SEG_COUNT"]
    parameters["SEG_HDR_WIDTH"] = 128
    parameters["SEG_PRFX_WIDTH"] = 32
    parameters["SEG_DATA_PAR_WIDTH"] = parameters["SEG_DATA_WIDTH"] // 32
    parameters["SEG_HDR_PAR_WIDTH"] = parameters["SEG_HDR_WIDTH"] // 32
    parameters["SEG_PRFX_PAR_WIDTH"] = parameters["SEG_PRFX_WIDTH"] // 32
    parameters["SEG_EMPTY_WIDTH"] = ((parameters["SEG_DATA_WIDTH"] // 32) - 1).bit_length()
    parameters["SEG_BAR_WIDTH"] = 3

    extra_env = {f"PARAM_{k}": str(v) for k, v in parameters.items()}

    sim_build = os.path.join(
        tests_dir,
        "sim_build",
        request.node.name.replace("[", "-").replace("]", ""),
    )

    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=verilog_sources,
        toplevel=toplevel,
        module=module,
        parameters=parameters,
        sim_build=sim_build,
        extra_env=extra_env,
    )
