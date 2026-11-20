# Unit test for src/clk_div.v — run with: make
SIM ?= icarus
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES = $(PWD)/../../../src/clk_div.v
TOPLEVEL = clk_div
COCOTB_TEST_MODULES = test_clk_div
include $(shell cocotb-config --makefiles)/Makefile.sim
