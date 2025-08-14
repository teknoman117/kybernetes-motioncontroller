# List of all the USFSMAX device files.
USFSMAXSRC := $(PWD)/drivers/usfsmax/USFSMAX.cpp

# Required include directories
USFSMAXINC := $(PWD)/drivers/usfsmax

# Shared variables
ALLCPPSRC += $(USFSMAXSRC)
ALLINC  += $(USFSMAXINC)