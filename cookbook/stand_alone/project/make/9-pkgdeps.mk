ifeq ($(os),Linux)
pkgcmd := apt install -y
endif

ifeq ($(os),Darwin)
#https://brew.sh/
pkgcmd := brew install
endif

ifeq ($(os),MinGW)
#https://www.msys2.org/
# no need to prefix for bison and flex because we need msys2 version pkgdeps := $(addprefix mingw-w64-x86_64-, $(pkgdeps))
pkgcmd := pacman -Suy --needed
endif

# external libraries
ifneq ($(strip $(pkdeps)),)
CXXFLAGS += $(shell pkg-config --cflags $(pkdeps))
LIBS += $(shell pkg-config --libs $(pkgdeps))
endif

install-pkgdeps:
	$(pkgcmd) $(pkgdeps)
.PHONY: install-pkgdeps
