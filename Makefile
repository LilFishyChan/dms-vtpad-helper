CC      ?= cc
VERSION ?= 0.1.0
CFLAGS  ?= -O2 -Wall -Wextra -std=c11
CFLAGS  += -DVTPAD_VERSION=\"$(VERSION)\" 
PREFIX  ?= /usr/local

BINS = dms-vtpad-helper vtpadctl

all: $(BINS)

dms-vtpad-helper: dms-vtpad-helper.c
	$(CC) $(CFLAGS) -o $@ $<

vtpadctl: vtpadctl.c
	$(CC) $(CFLAGS) -o $@ $<

install: all
	install -Dm755 dms-vtpad-helper $(DESTDIR)$(PREFIX)/bin/dms-vtpad-helper
	install -Dm755 vtpadctl $(DESTDIR)$(PREFIX)/bin/vtpadctl

clean:
	rm -f $(BINS)

.PHONY: all install clean
