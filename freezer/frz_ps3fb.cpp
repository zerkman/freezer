/* PS3 framebuffer environment.
 * Copyright (C) 2010 Francois Galea <fgalea@free.fr>
 *
 * This file is part of Freezer.
 *
 * Freezer is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * Freezer is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Freezer.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include <linux/kd.h>
#include <linux/fb.h>
#include <asm/ps3fb.h>
#include <linux/joystick.h>

#include "frz_ps3fb.hpp"

using namespace Frz;

Buttons::Buttons() {
  int i;
  for (i = 0; i < int(sizeof(buttons)/sizeof(buttons[0])); i++)
    buttons[i] = 0;
}

int Buttons::getButton(int b) {
  update();
  return getButtonNoUpdate(b);
}

int Buttons::getButtonNoUpdate(int b) {
  return (buttons[b/sizeof(unsigned int)]&(1<<(b%sizeof(unsigned int))))?1:0;
}

Joystick::Joystick() : Buttons() {
  const static char *devs[] = {
    "/dev/js0", "/dev/input/js2", "/dev/input/js1", "/dev/input/js0" };
  int i;
  int size = sizeof(devs)/sizeof(devs[0]);
  joyFD = -1;
  for (i=0; joyFD<0 && i<size; i++)
    joyFD = open(devs[i], O_RDONLY);
  if (joyFD<0) {
    std::cerr<<"unable to open joystick device"<<std::endl;
    exit(1);
  }
  std::cout << "successfully opened joystick device "<<devs[i-1]<<std::endl;
  fcntl(joyFD, F_SETFL, O_NONBLOCK);

  for (int i=0; i<4; i++)
    axis[i] = 0;
}

void Joystick::update() {
  struct js_event js;
  int r=0;
  while ((r=read(joyFD, &js, sizeof(struct js_event))) >0) {
    if (js.type & JS_EVENT_BUTTON) {
      if (js.value)
        buttons[0] |= (1<<js.number);
      else
        buttons[0] &=~ (1<<js.number);
    }
    if (js.type & JS_EVENT_AXIS && js.number<4)
      axis[js.number] = js.value;
  }
}

int Joystick::getAllButtons() {
  update();
  return buttons[0];
}

int Joystick::getAxis(int a) {
  update();
  return axis[a];
}

Joystick::~Joystick() {
  close(joyFD);
}

Keyboard::Keyboard() : Buttons(), fd(0), e0(0) {
  struct termios newtcattr;
  std::string env = getenv("TERM");

  if (env.compare("linux")) {
    std::cerr<<"this is not a linux terminal"<<std::endl;
    exit(1);
  }
  if (ioctl(fd, KDGKBMODE, &oldkbmode)) {
    perror("KDGKBMODE");
    exit(1);
  }
  if (tcgetattr(fd, &oldtcattr) == -1)
    perror("tcgetattr");
  if (tcgetattr(fd, &newtcattr) == -1)
    perror("tcgetattr");
  newtcattr.c_lflag &= ~ (ICANON | ECHO | ISIG);
  newtcattr.c_iflag = 0;
  newtcattr.c_iflag = 0;
  newtcattr.c_cc[VMIN] = 0;
  newtcattr.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSAFLUSH, &newtcattr) == -1)
    perror("tcsetattr");

  ioctl(fd, KDSKBMODE, K_RAW);
  fcntl(fd, F_SETFL, O_NONBLOCK);
}

Keyboard::~Keyboard() {
  if (ioctl(fd, KDSKBMODE, oldkbmode)) {
    perror("KDSKBMODE");
    exit(1);
  }
  if (tcsetattr(fd, 0, &oldtcattr) == -1)
    perror("tcsetattr");
  close(fd);
}

void Keyboard::update() {
  unsigned char code;
  int r=0, c, c0, c1;
  while ((r=read(0, &code, 1)) >0) {
    switch (code) {
    case 0xe0:
      e0 = 0x80;
    case 0xe1:
      continue;
    default:
      c = (code & 0x7f) + e0;
      c0 = c / sizeof(unsigned int);
      c1 = c % sizeof(unsigned int);
      if (code & 0x80)
        buttons[c0] &= ~(1<<c1);
      else
        buttons[c0] |= (1<<c1);
      e0 = 0;
    }
  }
}

int Keyboard::getAllButtons() {
  update();
  int i, but = 0, mask = 1;
  static const int keycodes[] = {
    0x0f, /* Tab */
    0x38, /* Left Alt */
    0xb8, /* Right Alt */
    0x1c, /* Enter */
    0xc8, /* Up */
    0xcd, /* Right */
    0xd0, /* Down */
    0xcb, /* Left */
    0x2a, /* Left Shift */
    0x36, /* Right Shift */
    0x1d, /* Left Control */
    0x9d, /* Right Control */
    0x3b, /* F1 */
    0x01, /* Esc */
    0x39, /* Space */
    0x3c, /* F2 */
  };
  for (i = 0; i < (int)(sizeof(keycodes)/sizeof(keycodes[0])); i++) {
    if (getButtonNoUpdate(keycodes[i]))
      but |= mask;
    mask <<= 1;
  }
  return but;
}

FrameBuffer::FrameBuffer() {
  struct ps3fb_ioctl_res res;

  consoleFD = open("/dev/fb0", O_RDWR);
  if (consoleFD<0) {
    std::cerr<<"Unable to open /dev/fb0"<<std::endl;
    exit(1);
  }

  if (ioctl(consoleFD, PS3FB_IOCTL_SCREENINFO, (unsigned long)&res)==-1) {
    std::cerr<<"framebuffer ioctl error"<<std::endl;
    exit(1);
  }
    
  if (res.num_frames<2) {
    std::cerr<<"Insufficient number of frames"<<std::endl;
    exit(1);
  }

  width = res.xres;
  height = res.yres;

  left = res.xoff;
  top = res.yoff;

  right = width-left;
  bottom = height-top;

  length = width * height * 4 * res.num_frames;
  addr = mmap(0, length, PROT_WRITE, MAP_SHARED, consoleFD, 0);
  addr2 = (void*) ( ((char*)addr) + width*height*4 );
  frame = false;

  std::cout<<"Screen resolution is "<<width<<"x"<<height<<", viewport ("<<left
      <<","<<top<<")-("<<right<<","<<bottom<<")"<<std::endl;
  std::cout<<"Buffers are at "<<addr<<" and "<<addr2<<", total length is"
      <<length<<std::endl;

  ioctl(consoleFD, PS3FB_IOCTL_ON, 0);
  memset(addr, 0, length);

  flip();

  gettimeofday(&tv, NULL);
  framecount = 0;

  switchMode(true);
}

FrameBuffer::~FrameBuffer() {
  ioctl(consoleFD, PS3FB_IOCTL_OFF, 0);
  munmap(addr, length);
  close(consoleFD);
  switchMode(false);
}

void FrameBuffer::switchMode(bool gfxMode) {
  const static char curs_off[] = { 27, '[', '?', '1', 'c' };
  const static char curs_on[] = { 27, '[', '?', '0', 'c' };
  int fd = open("/dev/console", O_NONBLOCK);
  if (fd >= 0) {
    ioctl(fd, KDSETMODE, gfxMode ? KD_GRAPHICS : KD_TEXT);
    close(fd);
  }
  write(0, gfxMode ? curs_off : curs_on, 5);
}

void* FrameBuffer::getFrameBuffer() {
  return current;
}

void* FrameBuffer::getOnScreenFrameBuffer() {
  return frame ? addr2 : addr;
}

void FrameBuffer::flip() {
  uint32_t framesel = frame ? 1 : 0;
  ioctl(consoleFD, PS3FB_IOCTL_FSEL, (unsigned long)&framesel);
  frame = !frame;
  current = frame ? addr : addr2;
  framecount++;
}

void FrameBuffer::vsync() {
  uint32_t crt = 0;
  ioctl(consoleFD, FBIO_WAITFORVSYNC, (unsigned long)&crt);
} 

void FrameBuffer::initFPS() {
  framecount = 0;
  gettimeofday(&tv, NULL);
}

float FrameBuffer::getFPS() {
  struct timeval tv2;
  gettimeofday(&tv2, NULL);

  int seconds = tv2.tv_sec - tv.tv_sec;
  int usec = tv2.tv_usec - tv.tv_usec;
  seconds += (usec<0) ? -1 : 0;
  usec += (usec<0) ? 1000000 : 0;

  float normalised = ((float)seconds) + ((float)usec)/1000000;
  return framecount/normalised;
}

int FrameBuffer::getFramecount() {
  return framecount;
}

PS3FBSystem::PS3FBSystem(Script &s, FrameBuffer &_fb, Buttons &_b):
    System(s, _fb.width, _fb.height), fb(_fb), b(_b), _pause(false) {
}

int PS3FBSystem::operator()() {
  bool pause_but;
  int but = 0;
  int msec0, msec1, msec;
  int delay = 0;
  struct timeval tv;
  struct timezone tz;
  void * screen;

  gettimeofday(&tv, &tz);
  msec0 = tv.tv_sec * 1000 + tv.tv_usec/1000;
  msec1 = 0;
  delay = 0;
  fb.initFPS();
  while (! (but&(1<<JOYSTICK_CIRCLE))) {
    screen = fb.getFrameBuffer();
    if (((uint64_t)screen) & 0x0FLL) {
      std::cerr<<"Unaligned screen buffer: "<<screen<<std::endl;
      exit(1);
    }

    gettimeofday(&tv, &tz); 
    msec = tv.tv_sec * 1000 + tv.tv_usec/1000;
    pause_but = (but & 1<<JOYSTICK_START);
    if (pause_but && !_pause)
      msec1 = msec;
    else if (!pause_but && _pause)
      delay += msec-msec1;
    _pause = pause_but;

    draw(_pause ? (msec1-msec0-delay) : (msec-msec0-delay), screen);

    fb.flip();
    fb.vsync();

    but = b.getAllButtons();
  }
  std::cout<<"FPS="<<fb.getFPS()<<std::endl;
  return 0;
}


