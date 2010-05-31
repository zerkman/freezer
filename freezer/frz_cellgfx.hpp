/* Cell Broadband Engine subsystem.
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
/*
 * frz_cellgfx.hpp - Cell BE subsystem for the Freezer animation system
 * by François Galea <fgalea@free.fr>
 */

#ifndef _EFFECT_SPE_HPP_
#define _EFFECT_SPE_HPP_

#include <libspe2.h>
#include <pthread.h>
#include "frz_pixbuf.hpp"

typedef int qword __attribute__((vector_size(16)));

#include "spu_engine.h"

#define SPE_COUNT 6

namespace Frz {

/*! \brief Cell Broadband Engine sybsystem class.
 *
 * This class interfaces with the libspe2 library. It creates and manages the
 * SPE threads, and implements mailbox-based communication with the different
 * SPE threads.
 */
class CellGfx : public Pixbuf
{
public:
  /*! \brief Constructs a Cell Broadband Engine sybsystem object.
   *
   * \param w Picture width in pixels.
   * \param h Picture height in pixels.
   * \param allocPixbuf if \b true, allocate a screen buffer.
   */
  CellGfx(int w, int h, bool allocPixbuf = false);
  ~CellGfx();

protected:
  /*! \brief Writes a set of 32-bit integers to the inbound mailbox of a
   * specified SPE thread.
   *
   * \param i SPE thread number (between \c 0 and <tt>SPE_COUNT-1</tt>).
   * \param array Pointer to the first element in the integer array.
   * \param count Number of elements to be written.
   */
  void in_mbox_write(int i, uint32_t* array, int count);
  /*! \brief Returns the status of the outbound mailbox of a
   * specified SPE thread.
   *
   * \param i SPE thread number (between \c 0 and <tt>SPE_COUNT-1</tt>).
   * \return the number of elements to be read in the mailbox.
   */
  int out_mbox_status(int i) {
    return spe_out_mbox_status(thr[i].context);
  }
  /*! \brief Reads a 32-bit integer from the outbound mailbox of a
   * specified SPE thread.
   *
   * \param i SPE thread number (between \c 0 and <tt>SPE_COUNT-1</tt>).
   * \return the read value or zero if no value is available.
   */
  uint32_t out_mbox_read(int i) {
    uint32_t ret;
    spe_out_mbox_read(thr[i].context, &ret, 1);
    return ret;
  }
  /*! \brief Reads a 32-bit integer from the interrupt outbound mailbox of a
   * specified SPE thread.
   *
   * If no value is available, the current thread waits for a value to become
   * available.
   *
   * \param i SPE thread number (between \c 0 and <tt>SPE_COUNT-1</tt>).
   * \return the read value.
   */
  uint32_t out_intr_mbox_read(int i=0) {
    uint32_t ret;
    spe_out_intr_mbox_read(thr[i].context, &ret, 1, SPE_MBOX_ALL_BLOCKING);
    return ret;
  }

private:
  struct ThrData {
    pthread_t id;
    spe_context_ptr_t context;
    spe_data data __attribute((aligned(16)));
  };

  ThrData thr[SPE_COUNT] __attribute((aligned(16)));
  static void * thread_main(void * arg);
};

}; // namespace Frz

#endif
