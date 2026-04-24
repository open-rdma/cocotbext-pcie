import logging

import pytest

from cocotbext.pcie.core.tlp import Tlp, TlpType
from cocotbext.pcie.core.utils import PcieId
from cocotbext.pcie.intel.rtile.interface import RTilePcieFrame, RTilePcieSink


def make_sink(bdf=None):
    sink = object.__new__(RTilePcieSink)
    sink.log = logging.getLogger("test.rtile.requester_id_fill")
    sink.bdf = bdf
    return sink


def make_mem_read64():
    tlp = Tlp()
    tlp.fmt_type = TlpType.MEM_READ_64
    tlp.length = 1
    tlp.first_be = 0xF
    tlp.last_be = 0x0
    tlp.tag = 0x12
    tlp.address = 0x1000
    tlp.requester_id = PcieId(0, 0, 0)
    return tlp


def make_mem_write():
    tlp = Tlp()
    tlp.fmt_type = TlpType.MEM_WRITE
    tlp.length = 1
    tlp.first_be = 0xF
    tlp.last_be = 0x0
    tlp.tag = 0x34
    tlp.address = 0x2000
    tlp.requester_id = PcieId(0, 0, 0)
    tlp.set_data(b"\x11\x22\x33\x44")
    return tlp


def make_cfg_read0():
    tlp = Tlp()
    tlp.fmt_type = TlpType.CFG_READ_0
    tlp.length = 1
    tlp.first_be = 0xF
    tlp.last_be = 0x0
    tlp.tag = 0x56
    tlp.address = 0x10
    tlp.requester_id = PcieId(0, 0, 0)
    tlp.completer_id = PcieId(0, 1, 0)
    return tlp


@pytest.mark.parametrize(
    "tlp_factory",
    [make_mem_read64, make_mem_write, make_cfg_read0],
)
def test_fill_requester_id_for_request_tlps(tlp_factory):
    expected_bdf = PcieId(0, 1, 0)
    sink = make_sink(expected_bdf)

    frame = RTilePcieFrame.from_tlp(tlp_factory())
    original_hdr_par = frame.hdr_par

    sink._fill_requester_id(frame)

    updated_tlp = frame.to_tlp()

    assert updated_tlp.requester_id == expected_bdf
    assert frame.check_parity()
    assert frame.hdr_par != original_hdr_par or expected_bdf == PcieId(0, 0, 0)


def test_completion_requester_id_is_not_overwritten():
    req = make_mem_read64()
    req.requester_id = PcieId(0, 2, 0)
    cpl = Tlp.create_completion_data_for_tlp(req, PcieId(0, 1, 0))
    cpl.byte_count = 4
    cpl.lower_address = 0
    cpl.set_data(b"\xaa\xbb\xcc\xdd")

    frame = RTilePcieFrame.from_tlp(cpl)
    sink = make_sink(PcieId(0, 3, 0))

    sink._fill_requester_id(frame)

    updated_tlp = frame.to_tlp()

    assert updated_tlp.fmt_type == TlpType.CPL_DATA
    assert updated_tlp.requester_id == req.requester_id
    assert frame.check_parity()


def test_requester_tlp_without_bdf_fails_fast():
    sink = make_sink()
    frame = RTilePcieFrame.from_tlp(make_mem_read64())

    with pytest.raises(RuntimeError, match="BDF is known"):
        sink._fill_requester_id(frame)
