create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]
create_clock -name clk_25m -period 40.000 [get_ports {clk_25m}]

# Clocks are crossed only through explicit CDC structures (dual-port RAM + synchronizers).
set_clock_groups -asynchronous -group [get_clocks {clk_50m}] -group [get_clocks {clk_25m}]
