/*ckwg +29
 * Copyright 2019 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "LoggerView.h"

#include "ui_LoggerView.h"

namespace kv = kwiver::vital;

//-----------------------------------------------------------------------------
class LoggerViewPrivate
{
public:

  Ui::LoggerView UI;
};

QTE_IMPLEMENT_D_FUNC(LoggerView)


//-----------------------------------------------------------------------------
LoggerView::LoggerView(
  QWidget* parent, Qt::WindowFlags flags)
  : QWidget{parent, flags}, d_ptr{new LoggerViewPrivate}
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  QFont font("Monospace", 10);
  font.setStyleHint(QFont::TypeWriter);
  d->UI.loggerText->document()->setDefaultFont(font);

  QObject::connect(this, &LoggerView::logMessage,
                   this, &LoggerView::appendMessage);
}

//-----------------------------------------------------------------------------
LoggerView::~LoggerView()
{
}

//-----------------------------------------------------------------------------
void LoggerView::logHandler(kv::kwiver_logger::log_level_t level,
                            std::string const& name,
                            std::string const& msg,
                            kv::logger_ns::location_info const& loc)
{
  std::string level_str = kv::kwiver_logger::get_level_string(level);
  auto full_msg = QString::fromStdString(
    "<b><font color=\"red\">" + level_str + "</font> " +
    name + "</b>: <pre>" + msg + "</pre>");
  emit logMessage(full_msg);
}

//-----------------------------------------------------------------------------
void LoggerView::appendMessage(QString const& msg)
{
  QTE_D();
  d->UI.loggerText->appendHtml(msg);
}
