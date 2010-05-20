/* Pixel buffer manager.
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
#include <fstream>

#include "frz_pixbuf.hpp"

Frz::Pixbuf::Pixbuf(int w, int h, bool allocPixbuf)
    : width(w), height(h), pixbuf(0)
{
  //std::cout << "Effect::Effect()" << std::endl;
  pitch = ((width*4)+0x7F) & 0x7FFFFF80;
  if (allocPixbuf) {
    void * buf;
    if (posix_memalign(&buf, 128, pitch*h))
    {
      std::cerr<<"Insufficient memory"<<std::endl;
      exit(1);
    }
    pixbuf = (uint32_t *)buf;
  }
}

Frz::Pixbuf::~Pixbuf() {
  if (pixbuf)
    free(pixbuf);
}

typedef struct {
  uint16_t bfType;      //must always be set to 'BM' to declare that this is a .bmp-file.
  uint32_t  bfSize;      //specifies the size of the file in bytes.
  uint16_t bfReserved1; //must always be set to zero.
  uint16_t bfReserved2; //must always be set to zero.
  uint32_t  bfOffBits;   //offset from the beginning of the file to the bitmap data.
} BitMapFileHeader;

typedef struct {
  uint32_t biSize; 	//size of the BITMAPINFOHEADER structure, in bytes.
  uint32_t biWidth; 	//width of the image, in pixels.
  uint32_t biHeight; 	//height of the image, in pixels.
  uint16_t biPlanes; 	//number of planes of the target device, must be set to zero.
  uint16_t biBitCount; 	//number of bits per pixel.
  uint32_t biCompression; 	//type of compression, usually set to zero (no compression).
  uint32_t biSizeImage; 	//size of the image data, in bytes. If there is no compression, it is valid to set this member to zero.
  uint32_t biXPelsPerMeter; //the horizontal pixels per meter on the designated targer device, usually set to zero.
  uint32_t biYPelsPerMeter; //the vertical pixels per meter on the designated targer device, usually set to zero.
  uint32_t biClrUsed; 	//number of colors used in the bitmap, if set to zero the number of colors is calculated using the biBitCount member.
  uint32_t biClrImportant; //number of color that are 'important' for the bitmap, if set to zero, all colors are important.
} BitMapInfoHeader;

static void write_le(uint32_t *p, uint32_t x) {
  unsigned char *a = (unsigned char *)p;
  a[0] = x;
  a[1] = x>>8;
  a[2] = x>>16;
  a[3] = x>>24;
}

static void write_le(uint16_t *p, uint16_t x) {
  unsigned char *a = (unsigned char *)p;
  a[0] = x;
  a[1] = x>>8;
}

void Frz::Pixbuf::saveBitmap(const char *filename) {
  unsigned char linebuf[width*3], *dst;
  uint32_t *src;
  int i, j;
  std::ofstream out(filename);

  BitMapFileHeader bmfh;
  BitMapInfoHeader bmih;
  write_le(&bmfh.bfType, 19778);
  write_le(&bmfh.bfSize, width*height*3+14+40);
  bmfh.bfReserved1 = 0;
  bmfh.bfReserved2 = 0;
  write_le(&bmfh.bfOffBits, 14+40);

  write_le(&bmih.biSize, 40);
  write_le(&bmih.biWidth, width);
  write_le(&bmih.biHeight, height);
  write_le(&bmih.biPlanes, 1);
  write_le(&bmih.biBitCount, 24);
  bmih.biCompression = 0;
  write_le(&bmih.biSizeImage, width*height*3);
  write_le(&bmih.biXPelsPerMeter, 3780);
  write_le(&bmih.biYPelsPerMeter, 3780);
  write_le(&bmih.biClrUsed, 0x0);
  bmih.biClrImportant = 0;

  out.write((char*)&bmfh.bfType, sizeof(bmfh.bfType));
  out.write((char*)&bmfh.bfSize, sizeof(bmfh.bfSize));
  out.write((char*)&bmfh.bfReserved1, sizeof(bmfh.bfReserved1));
  out.write((char*)&bmfh.bfReserved2, sizeof(bmfh.bfReserved2));
  out.write((char*)&bmfh.bfOffBits, sizeof(bmfh.bfOffBits));
  out.write((char*)&bmih, sizeof(bmih));
  for (i=0; i<height; i++)
  {
    dst = linebuf;
    src = pixbuf + (height-i-1)*width;
    for (j=0; j<width; j++) {
      *dst++ = *src;
      *dst++ = (*src)>>8;
      *dst++ = (*src)>>16;
      src++;
    }
    out.write((char*)linebuf, width*3);
  }
}

