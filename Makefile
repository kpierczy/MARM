all:
	$(MAKE) -C isix_c_examples
	$(MAKE) -C isix_cpp_examples
	$(MAKE) -C advanced/usbhostkbd

clean:
	$(MAKE) -C advanced/usbhostkbd clean
	$(MAKE) -C isix_c_examples clean
	$(MAKE) -C isix_cpp_examples clean 

program:
	$(MAKE) -C advanced/usbhostkbd program

