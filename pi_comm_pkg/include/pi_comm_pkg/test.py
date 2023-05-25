#!/usr/bin/env python

import MCP3008 as MCP
import time


mcp = MCP.MCP3008()
time.sleep(2)

for a in range(8):
	output = mcp.read_analog_input(a)
	print(output)
	
mcp.close_SPI()
