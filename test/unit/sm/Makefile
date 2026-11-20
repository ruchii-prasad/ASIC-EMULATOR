# Unit tests for src/sm.v (with fifo + clk_div) — run with: make
SIM ?= icarus
TOPLEVEL_LANG ?= verilog
SRC = $(PWD)/../../../src
VERILOG_SOURCES = $(SRC)/clk_div.v $(SRC)/fifo.v $(SRC)/sm.v $(PWD)/sm_wrap.v
TOPLEVEL = sm_wrap
COCOTB_TEST_MODULES = test_sm
include $(shell cocotb-config --makefiles)/Makefile.sim
