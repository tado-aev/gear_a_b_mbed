# Copyright 2017 Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.

ifndef ROS_LIB_DIR
	$(error ROS_LIB_DIR env variable must be set.)
endif

ifndef GCC4MBED_DIR
	$(error GCC4MBED_DIR env variable must be set.)
endif

PROJECT			:= gear-a-b-mbed
DEVICES			:= LPC1768
GCC4MBED_DIR	:= $(GCC4MBED_DIR)
USER_LIBS		:= include !$(ROS_LIB_DIR) $(ROS_LIB_DIR)/BufferedSerial
MBED_OS_ENABLE	:= 0
NO_FLOAT_PRINTF	:= 1
NO_FLOAT_SCANF	:= 1

include $(GCC4MBED_DIR)/build/gcc4mbed.mk
