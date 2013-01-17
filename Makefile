PREFIX = /usr
LD = ld
CC = gcc
INSTALL = install
CFLAGS = -g -pipe -O2 -Wall -Wextra -pedantic
LDFLAGS =
VLC_PLUGIN_CFLAGS := $(shell pkg-config --cflags vlc-plugin)
VLC_PLUGIN_LIBS := $(shell pkg-config --libs vlc-plugin)

libdir = $(PREFIX)/lib
plugindir = $(libdir)/vlc/plugins

override CC += -std=gnu99
override CFLAGS += -DPIC -I.
override CFLAGS += -fPIC
override LDFLAGS += -Wl,-no-undefined,-z,defs

override CFLAGS += -DMODULE_STRING=\"h264-vaapi\"
override CFLAGS += $(VLC_PLUGIN_CFLAGS) $(shell pkg-config --cflags libva-x11 libva-drm x11)
override LDFLAGS += $(VLC_PLUGIN_LIBS) $(shell pkg-config --libs libva-x11 libva-drm x11)

TARGETS = libh264-vaapi-enc_plugin.so

all: libh264-vaapi-enc_plugin.so

install: all
	mkdir -p -- $(DESTDIR)$(plugindir)/codec
	$(INSTALL) --mode 0755 libh264-vaapi-enc_plugin.so $(DESTDIR)$(plugindir)/codec

install-strip:
	$(MAKE) install INSTALL="$(INSTALL) -s"

uninstall:
	rm -f $(plugindir)/codec/libh264-vaapi-enc_plugin.so

clean:
	rm -f -- libh264-vaapi-enc_plugin.so *.o

mostlyclean: clean

SOURCES = vlc-h264-vaapi-enc.c

libh264-vaapi-enc_plugin.so: $(SOURCES:%.c=%.o)
	$(CC) $(LDFLAGS) -shared -o $@ $^

.PHONY: all install install-strip uninstall clean mostlyclean

