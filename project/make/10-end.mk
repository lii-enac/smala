# --

distclean clear:
	rm -rf build .ninja_log
clean:
	rm -f $(gen_srcs) $(objs) $(deps)
.PHONY: clean clear distclean