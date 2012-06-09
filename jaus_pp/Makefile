include $(shell rospack find mk)/cmake.mk

all : install

VERSION  = 2.110519
PACKAGE  = JAUS++-$(VERSION)
TARBALL  = $(PACKAGE)-src.tar.gz
TARBALL_URL = http://sourceforge.net/projects/active-ist/files/JAUS%2B%2B/$(VERSION)/JAUS%2B%2B-$(VERSION)-src.tar.gz/download
SOURCE_DIR = build/$(PACKAGE)-src
UNPACK_CMD = tar xzf
#MD5SUM_FILE = $(FILENAME).md5sum
TARBALL_PATCH = jaus_pp.patch
include $(shell rospack find mk)/download_unpack_build.mk

MKDIR = mkdir
MV = mv
RM = rm -rf

configure : $(SOURCE_DIR)/unpacked
	$(MKDIR) -p $(SOURCE_DIR)/build && cd $(SOURCE_DIR)/build && cmake ../libraries/jaus++/2.0/build/cmake

build-it : configure
	$(MAKE) -C $(SOURCE_DIR)/build

bin :
	$(MKDIR) bin

lib :
	$(MKDIR) lib

include:
	$(MKDIR) include

install-lib   : build-it lib
	$(MV) $(SOURCE_DIR)/libraries/jaus++/2.0/lib/* lib/ && touch install-lib

install-bin   : build-it bin
	$(MV) $(SOURCE_DIR)/libraries/jaus++/2.0/bin/* bin/ && touch install-bin

install-inc   : build-it include
	$(MV) $(SOURCE_DIR)/libraries/jaus++/2.0/include/jaus include/ && touch install-inc

install-real: install-bin install-lib install-inc
	touch install

install : wipe
	$(MAKE) install-real

clean :
	$(RM) $(SOURCE_DIR) install lib bin

wipe : clean
