#!/usr/bin/env bash
set -xe

export UNIX_GFLAGS_DIR=/usr/local/Cellar/gflags/2.2.1

export UNIX_GLOG_DIR=/usr/local/Cellar/glog/0.3.5_3

export UNIX_PROTOBUF_DIR=/usr/local/Cellar/protobuf/3.5.1_1

export UNIX_CBC_DIR=/usr/local/Cellar/cbc/2.9.9_1
export UNIX_CLP_DIR=/usr/local/Cellar/clp/1.16.11
export UNIX_CGL_DIR=/usr/local/Cellar/cgl/0.59.10
export UNIX_OSI_DIR=/usr/local/Cellar/osi/0.107.9
export UNIX_COINUTILS_DIR=/usr/local/Cellar/coinutils/2.10.14

export UNIX_DYNAMIC_DEPENDENCIES=1

#make clean_third_party
make clean_cc
make cc
