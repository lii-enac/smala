
#----------------------------

$(objs): DJNN_CXXFLAGS += $(CXXFLAGS_CFG) $(djnn_cflags) $(smala_cflags) -I$(src_dir) -I$(build_dir)/$(src_dir)
	
$(exe): LDFLAGS += $(djnn_ldflags) $(smala_ldflags)
$(exe): LIBS += $(app_libs)


$(exe): $(objs)
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(CXXLD) $^ -o $@ $(LDFLAGS) $(LIBS)
else
	@$(call rule_message,linking to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(CXXLD) $^ -o $@ $(LDFLAGS) $(LIBS)
endif

# .sma to .cpp, .c etc
$(build_dir)/%.cpp $(build_dir)/%.h: %.sma
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(smalac) $(SMAFLAGS) -cpp $^ -builddir $(build_dir)
else
	@$(call rule_message,compiling to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(smalac) $(SMAFLAGS) -cpp $^ -builddir $(build_dir)
endif


# from .cpp user sources
$(build_dir)/%.o: %.cpp
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(CXX) $(DJNN_CXXFLAGS) -c $< -o $@
else
	@$(call rule_message,compiling to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(CXX) $(DJNN_CXXFLAGS) -c $< -o $@
endif

# from .c user sources
$(build_dir)/%.o: %.c
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@
else
	@$(call rule_message,compiling to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS) -c $< -o $@
endif


# for .cpp generated sources
$(build_dir)/%.o: $(build_dir)/%.cpp
ifeq ($V,max)
	@mkdir -p $(dir $@)
	$(CXX) $(DJNN_CXXFLAGS) -c $< -o $@
else
	@$(call rule_message,compiling to,$(stylized_target))
	@mkdir -p $(dir $@)
	@$(CXX) $(DJNN_CXXFLAGS) -c $< -o $@
endif

deps := $(objs:.o=.d)

ifneq ($(nodeps),1)
-include $(deps)
endif

ifeq ($(keep_intermediate),yes)
.SECONDARY:
endif

buildroot:
	./project/buildroot/make_buildroot.sh install $(original_exe)

burn:
	./project/buildroot/make_buildroot.sh burn $(original_exe)
