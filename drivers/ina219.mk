# List of all the INA219 device files.
INA219SRC := $(PWD)/drivers/ina219.c

# Required include directories
INA219INC := $(PWD)/drivers

# Shared variables
ALLCSRC += $(INA219SRC)
ALLINC  += $(INA219INC)