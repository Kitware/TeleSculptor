/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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

#include "OutputDialog.h"

#include "ui_OutputDialog.h"

#include <qtGradient.h>
#include <qtIndexRange.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>
#include <stdio.h>
#include <QProcess>

//BEGIN OutputDialogPrivate

//-----------------------------------------------------------------------------
class OutputDialogPrivate
{
public:

  Ui::OutputDialog UI;
  qtUiState uiState;

  QProcess *process;
};

QTE_IMPLEMENT_D_FUNC(OutputDialog)


//END OutputDialogPrivate


//BEGIN OutputDialog

//-----------------------------------------------------------------------------
OutputDialog::OutputDialog(QWidget* parent, Qt::WindowFlags flags)
  : QDialog(parent, flags), d_ptr(new OutputDialogPrivate)
{
  QTE_D();

  this->setAttribute(Qt::WA_DeleteOnClose);

  // Set up UI
  d->UI.setupUi(this);

}

//-----------------------------------------------------------------------------
OutputDialog::~OutputDialog()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void OutputDialog::setOutputToDisplay(QProcess *process)
{
  QTE_D();

  d->process = process;
}

//-----------------------------------------------------------------------------
void OutputDialog::ouputProcess()
{
  QTE_D();

    QString outputText(d->process->readAll());
    d->UI.plainTextEdit->appendPlainText(outputText);
}


//END OutputDialog
