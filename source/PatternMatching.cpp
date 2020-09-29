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

/* qt header */
#include <QApplication>
#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>

/* locl haeder */
#include "PatternMatching.h"

namespace vx {

  constexpr int defaultX = 640;
  constexpr int defaultY = 480;

  PatternMatching::PatternMatching( QWidget *parent )
    : QWidget( parent ) {

    QVBoxLayout *verticalLayout = new QVBoxLayout( this );

    m_txtStream = new QLabel( this );
    verticalLayout->addWidget( m_txtStream );

    resize( QSize( defaultX, defaultY ) );

    show();
  }

  void PatternMatching::updateStream( const QImage &_frame ) {

    QPixmap pixmap( m_txtStream->width(), m_txtStream->height() );

    QPainter painter( &pixmap );
    painter.drawImage( QRect( 0, 0,  m_txtStream->width(), m_txtStream->height() ), _frame, QRect( 0, 0, _frame.width(), _frame.height() ) );

    QPixmap logo( QStringLiteral( "logo.png" ) );
    painter.drawPixmap( QRect( 0, m_txtStream->height() - ( logo.height() / 2 ), logo.width() / 2, logo.height() / 2 ), logo, QRect( 0, 0, logo.width(), logo.height() ) );
    painter.end();

    m_txtStream->setPixmap( pixmap );
    QApplication::processEvents();
  }
}
