// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_ABOUTDIALOG_H_
#define TELESCULPTOR_ABOUTDIALOG_H_

#include <qtGlobal.h>

#include <QDialog>

class AboutDialogPrivate;

class AboutDialog : public QDialog
{
  Q_OBJECT

public:
  explicit AboutDialog(QWidget* parent = nullptr, Qt::WindowFlags f = {});
  ~AboutDialog() override;

protected slots:
  void openLink(QString const&);

protected:
  QTE_DECLARE_PRIVATE_RPTR(AboutDialog)

private:
  QTE_DECLARE_PRIVATE(AboutDialog)
  Q_DISABLE_COPY(AboutDialog)
};

#endif
