/*
 * Copyright (c) 2020 Florian Becker <fb@vxapps.com> (VX APPS).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/* stl header */
#include <chrono>
#include <thread>

/**
 * @brief vx (VX Apps) namespace.
 */
namespace vx {

  /**
   * @brief Timing class for timeouts.
   * @author Florian Becker <fb\@vxapps.com> (VX Apps)
   */
  class Timer {

  public:
    /**
     * @brief Call a function after timeout.
     * @param _function   Call back function.
     * @param _delay   Delay after the function is valled in milliseconds.
     */
    template<typename Function>
    void setTimeout( Function _function, int _delay ) {

      this->m_clear = false;
      std::thread t( [ = ]() {

        if ( this->m_clear ) {

          return;
        }
        std::this_thread::sleep_for( std::chrono::milliseconds( _delay ) );
        if ( this->m_clear ) {

          return;
        }
        _function();
      } );
      t.detach();
    }

    /**
     * @brief Call a function after interval.
     * @param _function   Call back function.
     * @param _interval   Intervall after the function is called in milliseconds.
     */
    template<typename Function>
    void setInterval( Function _function, int _interval ) {

      this->m_clear = false;
      std::thread t( [ = ]() {

        while ( true ) {

          if ( this->m_clear ) {

            return;
          }
          std::this_thread::sleep_for( std::chrono::milliseconds( _interval ) );
          if ( this->m_clear ) {

            return;
          }
          _function();
        }
      } );
      t.detach();
    }

    /**
     * @brief Stoping the current timer to not exeecute the call back function.
     */
    inline void stop() { this->m_clear = true; }

  private:
    /**
     * @brief Clear the timer.
     */
    bool m_clear = false;
  };
}
