/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include "AboutDialog.h"
#include "ui_AboutDialog.h"

#include "Version.h"

#include <qtSaxNodes.h>
#include <qtSaxWriter.h>
#include <qtUtil.h>

#include <QtCore/QFile>

QTE_IMPLEMENT_D_FUNC(AboutDialog)

namespace // anonymous
{

//-----------------------------------------------------------------------------
QString formatTitle(QString formatStr)
{
  formatStr.replace("@APP_TITLE@", QApplication::applicationName());
  formatStr.replace("@APP_VERSION@", QApplication::applicationVersion());
  formatStr.replace("@QT_VERSION@", QString::fromLocal8Bit(qVersion()));
  return formatStr;
}

//-----------------------------------------------------------------------------
QString buildCopyrightText()
{
  QString format = "Copyright &copy;%1 %2";
  return format.arg(MAPTK_COPYRIGHT_YEAR).arg(QApplication::organizationName());
}

//-----------------------------------------------------------------------------
QString loadText(QString const& resource)
{
  QFile f(resource);
  f.open(QIODevice::ReadOnly);
  return f.readAll();
}

//-----------------------------------------------------------------------------
QString loadMarkdown(QString const& resource)
{
  auto input = loadText(resource);
  input.replace("&", "&amp;");
  input.replace("<", "&lt;");
  input.replace(">", "&gt;");
  input.replace("``", "&ldquo;");
  input.replace("''", "&rdquo;");
  input.replace("`", "&lsquo;");
  input.replace("'", "&rsquo;");

  QString markup;
  qtSaxWriter out(&markup);
  out << qtSaxElement("html") << qtSaxElement("body");

  auto depth = 2;
  foreach (auto const& block, input.split("\n\n"))
  {
    auto const text = block.simplified();
    if (text.startsWith("*"))
    {
      if (depth < 3)
      {
        out << qtSaxElement("ul")
            << qtSaxAttribute("style", "margin-left: 1.5em;"
                                       "-qt-list-indent: 0;");
        ++depth;
      }
      out << qtSaxElement("li")
          << qtSaxAttribute("style", "margin-bottom: 1em;")
          << qtSaxText(text.mid(1).trimmed(), qtSaxText::TextWithEntities)
          << qtSax::EndElement;
    }
    else
    {
      while (depth > 2)
      {
        out << qtSax::EndElement;
        --depth;
      }
      out << qtSaxElement("p")
          << qtSaxText(text, qtSaxText::TextWithEntities)
          << qtSax::EndElement;
    }
  }
  while (depth)
  {
    out << qtSax::EndElement;
    --depth;
  }

  return markup;
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class AboutDialogPrivate
{
public:
  Ui::AboutDialog UI;
};

//-----------------------------------------------------------------------------
AboutDialog::AboutDialog(QWidget* parent, Qt::WindowFlags f)
  : QDialog(parent, f), d_ptr(new AboutDialogPrivate)
{
  QTE_D(AboutDialog);

  // Set up UI
  d->UI.setupUi(this);
  qtUtil::setStandardIcons(d->UI.buttonBox);
  this->setWindowTitle(QString("About %1").arg(qApp->applicationName()));

  // Fill title and copyright texts
  d->UI.title->setText(formatTitle(d->UI.title->text()));
  d->UI.copyright->setText(buildCopyrightText());

  // Load various supplemental texts
  d->UI.acknowledgments->setText(loadMarkdown(":/ACKNOWLEDGMENTS"));
  d->UI.license->setHtml(loadMarkdown(":/LICENSE"));
  d->UI.buildInfo->setHtml(loadText(":/BUILDINFO"));
}

//-----------------------------------------------------------------------------
AboutDialog::~AboutDialog()
{
}
