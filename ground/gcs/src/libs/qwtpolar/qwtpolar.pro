# -*- mode: sh -*- ##############################################
# QwtPolar Widget Library
# Copyright (C) 2008   Uwe Rathmann
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the Qwt License, Version 1.0
#################################################################

include( qwtpolarconfig.pri )

TEMPLATE = subdirs

CONFIG  += warn_off
CONFIG  += ordered
SUBDIRS = src
