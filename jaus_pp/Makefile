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
CMAKE = cmake

configure : $(SOURCE_DIR)/unpacked
	$(MKDIR) -p $(SOURCE_DIR)/build && cd $(SOURCE_DIR)/build && $(CMAKE) ../libraries/jaus++/2.0/build/cmake -DCMAKE_INSTALL_PREFIX="$(PWD)"

build-it : configure
	$(MAKE) -C $(SOURCE_DIR)/build

install-real:
	$(MAKE) -C $(SOURCE_DIR)/build install
	touch install

install : wipe
	$(MAKE) install-real

clean :
	$(RM) $(SOURCE_DIR) install lib bin

wipe : clean
