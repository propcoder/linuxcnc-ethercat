# We let this file know where halcompile is. In our case linuxcnc is in the /opt/linuxcnc direcory.
COMP=/opt/linuxcnc/bin/halcompile 

.PHONY: configure
configure:
	@echo "COMP = $(COMP)"
	@echo "MODINC = $(MODINC)"

# include modinc
MODINC=$(shell $(COMP) --print-modinc)
ifeq (, $(MODINC))
  $(error Unable to get modinc path)
endif

include $(MODINC)

