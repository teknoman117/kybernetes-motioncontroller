# List of all the USFSMAX device files.
USFSMAXSRC := $(PWD)/drivers/USFSMAXv1/USFSMAX.cpp

# Required include directories
USFSMAXINC := $(PWD)/drivers/USFSMAXv1

# Shared variables
ALLCPPSRC += $(USFSMAXSRC)
ALLINC  += $(USFSMAXINC)