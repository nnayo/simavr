# 
# 	Copyright 2008-2012 Michel Pollet <buserror@gmail.com>
# 	modified by Yann GOUY <yann_gouy@yahoo.fr>
#
#	This file is part of simavr.
#
#	simavr is free software: you can redistribute it and/or modify
#	it under the terms of the GNU General Public License as published by
#	the Free Software Foundation, either version 3 of the License, or
#	(at your option) any later version.
#
#	simavr is distributed in the hope that it will be useful,
#	but WITHOUT ANY WARRANTY; without even the implied warranty of
#	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#	GNU General Public License for more details.
#
#	You should have received a copy of the GNU General Public License
#	along with simavr.  If not, see <http://www.gnu.org/licenses/>.

targets = mpu6050.so eeprom_i2c.so sc18is600.so gl_lcd.so
simavr = ../..

IPATH = .
IPATH += ${simavr}/include
IPATH += ${simavr}/simavr/sim
IPATH += ${simavr}/examples/parts

VPATH = .
VPATH += ${simavr}/examples/shared
VPATH += ${simavr}/examples/parts

CFLAGS += -shared -O0 -g
CPPFLAGS    += ${shell pkg-config --cflags glu}
#LDFLAGS     += ${shell pkg-config --libs glu} -lglut
LDFLAGS     += -lglut
LDFLAGS		+= -shared -lpthread


all: ${targets} 

include ${simavr}/Makefile.common

%.so: %.c
	$(CC) -MMD ${CPPFLAGS} ${CFLAGS}  ${LFLAGS} -o $@ $^


gl_lcd.so: gl_lcd.o ../parts/ac_input.o ../parts/hd44780_glut.o ../parts/hd44780.o
	$(CC) -MMD ${LDFLAGS} -o $@ $^

clean:
	rm -rf *.so .*.swo .*.swp .*.swm .*.swn


