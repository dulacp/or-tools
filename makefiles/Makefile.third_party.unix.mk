.PHONY: help_third_party # Generate list of Prerequisite targets with descriptions.
help_third_party:
	@echo Use one of the following Prerequisite targets:
	@grep "^.PHONY: .* #" $(CURDIR)/makefiles/Makefile.third_party.unix.mk | sed "s/\.PHONY: \(.*\) # \(.*\)/\1\t\2/" | expand -t20
	@echo

# Checks if the user has overwritten default libraries and binaries.
UNIX_GFLAGS_DIR ?= $(OR_TOOLS_TOP)/dependencies/install
UNIX_GLOG_DIR ?= $(OR_TOOLS_TOP)/dependencies/install
UNIX_PROTOBUF_DIR ?= $(OR_TOOLS_TOP)/dependencies/install
UNIX_CBC_DIR ?= $(OR_ROOT_FULL)/dependencies/install
UNIX_CLP_DIR ?= $(UNIX_CBC_DIR)

# Unix specific definitions
PROTOBUF_DIR = $(UNIX_PROTOBUF_DIR)
UNIX_SWIG_BINARY ?= swig
SWIG_BINARY = $(UNIX_SWIG_BINARY)

# Tags of dependencies to checkout.
GFLAGS_TAG = 2.2.1
PROTOBUF_TAG = 3.5.1
GLOG_TAG = 0.3.5
CBC_TAG = 2.9.9
PATCHELF_TAG = 0.9

# Detect if patchelf is needed
ifeq ($(PLATFORM), LINUX)
    PATCHELF=dependencies/install/bin/patchelf
endif

# Main target.
.PHONY: third_party # Build OR-Tools Prerequisite
third_party: makefile_third_party install_third_party

.PHONY: third_party_check # Check if "make third_party" have been run or not
third_party_check:
ifeq ($(wildcard $(UNIX_GFLAGS_DIR)/include/gflags/gflags.h),)
	$(error Third party GFlags files was not found! did you run 'make third_party' or set UNIX_GFLAGS_DIR ?)
endif
ifeq ($(wildcard $(UNIX_GLOG_DIR)/include/glog/logging.h),)
	$(error Third party GLog files was not found! did you run 'make third_party' or set UNIX_GLOG_DIR ?)
endif
ifeq ($(wildcard $(UNIX_PROTOBUF_DIR)/include/google/protobuf/descriptor.h),)
	$(error Third party Protobuf files was not found! did you run 'make third_party' or set UNIX_PROTOBUF_DIR ?)
endif
ifeq ($(wildcard $(UNIX_CBC_DIR)/include/cbc/coin/CbcModel.hpp $(UNIX_CBC_DIR)/include/coin/CbcModel.hpp),)
	$(error Third party Cbc files was not found! did you run 'make third_party' or set UNIX_CBC_DIR ?)
endif
ifeq ($(wildcard $(UNIX_CLP_DIR)/include/clp/coin/ClpSimplex.hpp $(UNIX_CLP_DIR)/include/coin/ClpSimplex.hpp),)
	$(error Third party Clp files was not found! did you run 'make third_party' or set UNIX_CLP_DIR ?)
endif

.PHONY: install_third_party
install_third_party: \
 archives_directory \
 install_directories \
 install_cbc \
 install_gflags \
 install_glog \
 install_protobuf

.PHONY: archives_directory
archives_directory:
	$(MKDIR_P) dependencies$Sarchives

.PHONY: install_directories
install_directories: dependencies/install/bin dependencies/install/lib dependencies/install/include/coin

dependencies/install:
	$(MKDIR_P) dependencies$Sinstall

dependencies/install/bin: dependencies/install
	$(MKDIR_P) dependencies$Sinstall$Sbin

dependencies/install/lib: dependencies/install
	$(MKDIR_P) dependencies$Sinstall$Slib

dependencies/install/include: dependencies/install
	$(MKDIR_P) dependencies$Sinstall$Sinclude

dependencies/install/include/coin: dependencies/install/include
	$(MKDIR_P) dependencies$Sinstall$Sinclude$Scoin

# Install gflags. This uses cmake.
install_gflags: dependencies/install/include/gflags/gflags.h

dependencies/install/include/gflags/gflags.h: dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake/Makefile
	cd dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake && \
	$(SET_COMPILER) make -j 4 && make install
	touch $@

dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake/Makefile: dependencies/sources/gflags-$(GFLAGS_TAG)/CMakeLists.txt
	-mkdir dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake
	cd dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake && $(SET_COMPILER) \
	$(CMAKE) -D BUILD_SHARED_LIBS=OFF \
		 -D BUILD_STATIC_LIBS=ON \
	         -D CMAKE_INSTALL_PREFIX=../../../install \
		 -D CMAKE_CXX_FLAGS="-fPIC $(MAC_VERSION)" \
	         ..

dependencies/sources/gflags-$(GFLAGS_TAG)/CMakeLists.txt:
	git clone --quiet -b v$(GFLAGS_TAG) https://github.com/gflags/gflags.git dependencies/sources/gflags-$(GFLAGS_TAG)

# Install protocol buffers.
install_protobuf: dependencies/install/bin/protoc

dependencies/install/bin/protoc: dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build/Makefile
	cd dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build && $(SET_COMPILER) make -j 4 && make install

dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build/Makefile: dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/CMakeLists.txt
	-$(MKDIR) dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build
	cd dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build && \
	  $(CMAKE) -D CMAKE_INSTALL_PREFIX=../../../../install \
		   -D protobuf_BUILD_TESTS=OFF \
                   -D BUILD_SHARED_LIBS=OFF \
                   -D CMAKE_CXX_FLAGS="-fPIC $(MAC_VERSION)" \
	           ..

dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/CMakeLists.txt:
	git clone --quiet https://github.com/google/protobuf.git dependencies/sources/protobuf-$(PROTOBUF_TAG) && \
		cd dependencies/sources/protobuf-$(PROTOBUF_TAG) && \
		git checkout tags/v$(PROTOBUF_TAG) -b $(PROTOBUF_TAG)

# Install GLOG.
install_glog: dependencies/install/include/glog/logging.h

dependencies/install/include/glog/logging.h: dependencies/sources/glog-$(GLOG_TAG)/build_cmake/Makefile
	cd dependencies/sources/glog-$(GLOG_TAG)/build_cmake && $(SET_COMPILER) make -j 4 && make install
	touch $@

dependencies/sources/glog-$(GLOG_TAG)/build_cmake/Makefile: dependencies/sources/glog-$(GLOG_TAG)/CMakeLists.txt | install_gflags
	-$(MKDIR) dependencies/sources/glog-$(GLOG_TAG)/build_cmake
	cd dependencies/sources/glog-$(GLOG_TAG)/build_cmake && \
	  $(CMAKE) -D CMAKE_INSTALL_PREFIX=../../../install \
                   -D BUILD_SHARED_LIBS=OFF \
                   -D CMAKE_CXX_FLAGS="-fPIC $(MAC_VERSION)" \
                   -D CMAKE_PREFIX_PATH="$(OR_TOOLS_TOP)/dependencies/install" \
	           ..

dependencies/sources/glog-$(GLOG_TAG)/CMakeLists.txt:
	git clone --quiet -b v$(GLOG_TAG) https://github.com/google/glog.git dependencies/sources/glog-$(GLOG_TAG)

# Install Coin CBC.
install_cbc: dependencies/install/bin/cbc

dependencies/install/bin/cbc: dependencies/sources/Cbc-$(CBC_TAG)/Makefile
	cd dependencies/sources/Cbc-$(CBC_TAG) && $(SET_COMPILER) make -j 4 && $(SET_COMPILER) make install

dependencies/sources/Cbc-$(CBC_TAG)/Makefile: dependencies/sources/Cbc-$(CBC_TAG)/Makefile.in
	cd dependencies/sources/Cbc-$(CBC_TAG) && $(SET_COMPILER) ./configure --prefix=$(OR_ROOT_FULL)/dependencies/install --disable-bzlib --without-lapack --enable-static --with-pic --enable-cbc-parallel ADD_CXXFLAGS="-w -DCBC_THREAD_SAFE -DCBC_NO_INTERRUPT $(MAC_VERSION)"

CBC_ARCHIVE:=https://www.coin-or.org/download/source/Cbc/Cbc-${CBC_TAG}.tgz

dependencies/sources/Cbc-$(CBC_TAG)/Makefile.in:
	wget --quiet --no-check-certificate --continue -P dependencies/archives ${CBC_ARCHIVE} || (@echo wget failed to dowload $(CBC_ARCHIVE), try running 'wget -P dependencies/archives --no-check-certificate $(CBC_ARCHIVE)' then rerun 'make third_party' && exit 1)
	tar xzf dependencies/archives/Cbc-${CBC_TAG}.tgz -C dependencies/sources/

# Install patchelf on linux platforms.
dependencies/install/bin/patchelf: dependencies/sources/patchelf-$(PATCHELF_TAG)/Makefile
	cd dependencies/sources/patchelf-$(PATCHELF_TAG) && make && make install

dependencies/sources/patchelf-$(PATCHELF_TAG)/Makefile: dependencies/sources/patchelf-$(PATCHELF_TAG)/configure
	cd dependencies/sources/patchelf-$(PATCHELF_TAG) && ./configure --prefix=$(OR_ROOT_FULL)/dependencies/install

dependencies/sources/patchelf-$(PATCHELF_TAG)/configure:
	git clone --quiet -b $(PATCHELF_TAG) https://github.com/NixOS/patchelf.git dependencies/sources/patchelf-$(PATCHELF_TAG)
	cd dependencies/sources/patchelf-$(PATCHELF_TAG) && ./bootstrap.sh

# Install Java protobuf
dependencies/install/lib/protobuf.jar: dependencies/install/bin/protoc
	cd dependencies/sources/protobuf-$(PROTOBUF_TAG)/java && \
	  ../../../install/bin/protoc --java_out=core/src/main/java -I../src \
	  ../src/google/protobuf/descriptor.proto
	cd dependencies/sources/protobuf-$(PROTOBUF_TAG)/java/core/src/main/java && $(JAVAC_BIN) com/google/protobuf/*java
	cd dependencies/sources/protobuf-$(PROTOBUF_TAG)/java/core/src/main/java && $(JAR_BIN) cvf ../../../../../../../install/lib/protobuf.jar com/google/protobuf/*class

# Install C# protobuf

#create .snk file if strong named dll is required (this is the default behaviour)

.PHONY: clean_third_party # Clean everything. Remember to also delete archived dependencies, i.e. in the event of download failure, etc.
clean_third_party:
	-$(DEL) Makefile.local
	-$(DELREC) dependencies/archives/Cbc*
	-$(DELREC) dependencies/archives
	-$(DELREC) dependencies/sources/Cbc*
	-$(DELREC) dependencies/sources/coin-cbc*
	-$(DELREC) dependencies/sources/gflags*
	-$(DELREC) dependencies/sources/glog*
	-$(DELREC) dependencies/sources/glpk*
	-$(DELREC) dependencies/sources/google*
	-$(DELREC) dependencies/sources/mono*
	-$(DELREC) dependencies/sources/pcre*
	-$(DELREC) dependencies/sources/swig*
	-$(DELREC) dependencies/sources/protobuf*
	-$(DELREC) dependencies/sources/sparsehash*
	-$(DELREC) dependencies/sources/libtool*
	-$(DELREC) dependencies/sources/autoconf*
	-$(DELREC) dependencies/sources/automake*
	-$(DELREC) dependencies/sources/bison*
	-$(DELREC) dependencies/sources/flex*
	-$(DELREC) dependencies/sources/help2man*
	-$(DELREC) dependencies/sources/patchelf*
	-$(DELREC) dependencies/install

# Create Makefile.local
.PHONY: makefile_third_party
makefile_third_party: Makefile.local

Makefile.local: makefiles/Makefile.third_party.unix.mk
	-$(DEL) Makefile.local
	@echo Generating Makefile.local
	@echo JAVA_HOME = $(JAVA_HOME)>> Makefile.local
	@echo UNIX_PYTHON_VER = $(DETECTED_PYTHON_VERSION)>> Makefile.local
	@echo PATH_TO_CSHARP_COMPILER = $(DETECTED_MCS_BINARY)>> Makefile.local
	@echo DOTNET_INSTALL_PATH = $(DOTNET_INSTALL_PATH)>> Makefile.local
	@echo CLR_KEYFILE = bin/or-tools.snk>> Makefile.local
	@echo >> Makefile.local
	@echo "# Define UNIX_GLPK_DIR to point to a compiled version of GLPK to use it" >> Makefile.local
	@echo "# Define UNIX_SCIP_DIR to point to a compiled version of SCIP to use it ">> Makefile.local
	@echo "#   i.e.: <path>/scipoptsuite-4.0.1/scip" >> Makefile.local
	@echo "#   On Mac OS X, compile scip with: " >> Makefile.local
	@echo "#     make GMP=false READLINE=false TPI=tny" >> Makefile.local
	@echo "#   On Linux, compile scip with: " >> Makefile.local
	@echo "#     make GMP=false READLINE=false TPI=tny USRCFLAGS=-fPIC USRCXXFLAGS=-fPIC USRCPPFLAGS=-fPIC" >> Makefile.local
	@echo "# Define UNIX_GUROBI_DIR and GUROBI_LIB_VERSION to use Gurobi" >> Makefile.local
	@echo "# Define UNIX_CPLEX_DIR to use CPLEX" >> Makefile.local
	@echo >> Makefile.local
	@echo "# Define UNIX_GFLAGS_DIR, UNIX_PROTOBUF_DIR, UNIX_GLOG_DIR," >> Makefile.local
	@echo "# UNIX_CLP_DIR, UNIX_CBC_DIR, UNIX_SWIG_BINARY if you wish to " >> Makefile.local
	@echo "# use a custom version. " >> Makefile.local

.PHONY: detect_third_party # Show variables used to find third party
detect_third_party:
	@echo Relevant info on third party:
	@echo UNIX_GFLAGS_DIR = $(UNIX_GFLAGS_DIR)
	@echo GFLAGS_INC = $(GFLAGS_INC)
	@echo GFLAGS_LNK = $(GFLAGS_LNK)
	@echo UNIX_GLOG_DIR = $(UNIX_GLOG_DIR)
	@echo GLOG_INC = $(GLOG_INC)
	@echo GLOG_LNK = $(GLOG_LNK)
	@echo UNIX_PROTOBUF_DIR = $(UNIX_PROTOBUF_DIR)
	@echo PROTOBUF_DIR = $(PROTOBUF_DIR)
	@echo PROTOBUF_INC = $(PROTOBUF_INC)
	@echo PROTOBUF_LNK = $(PROTOBUF_LNK)
	@echo UNIX_CBC_DIR = $(UNIX_CBC_DIR)
	@echo CBC_INC = $(CBC_INC)
	@echo CBC_LNK = $(CBC_LNK)
	@echo UNIX_CLP_DIR = $(UNIX_CLP_DIR)
	@echo CLP_INC = $(CLP_INC)
	@echo CLP_LNK = $(CLP_LNK)
ifdef UNIX_GLPK_DIR
	@echo UNIX_GLPK_DIR = $(UNIX_GLPK_DIR)
	@echo GLPK_INC = $(GLPK_INC)
	@echo GLPK_LNK = $(GLPK_LNK)
endif
ifdef UNIX_SCIP_DIR
	@echo UNIX_SCIP_DIR = $(UNIX_SCIP_DIR)
	@echo SCIP_INC = $(SCIP_INC)
	@echo SCIP_LNK = $(SCIP_LNK)
endif
ifdef UNIX_CPLEX_DIR
	@echo UNIX_CPLEX_DIR = $(UNIX_CPLEX_DIR)
	@echo CPLEX_INC = $(CPLEX_INC)
	@echo CPLEX_LNK = $(CPLEX_LNK)
endif
ifdef UNIX_GUROBI_DIR
	@echo UNIX_GUROBI_DIR = $(UNIX_GUROBI_DIR)
	@echo GUROBI_INC = $(GUROBI_INC)
	@echo GUROBI_LNK = $(GUROBI_LNK)
endif
	@echo
