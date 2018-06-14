/*ckwg +29
 * Copyright 2018 by Kitware, Inc.
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

#include "MetadataView.h"

#include <vital/types/metadata_traits.h>

#include <vital/range/filter.h>
#include <vital/range/valid.h>

#include <qtColorUtil.h>
#include <qtIndexRange.h>
#include <qtSqueezedLabel.h>
#include <qtStlUtil.h>

#include <QEvent>
#include <QHash>
#include <QLabel>
#include <QScrollBar>
#include <QSet>
#include <QSpacerItem>
#include <QVariant>
#include <QVBoxLayout>

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

QTE_IMPLEMENT_D_FUNC(MetadataView)

///////////////////////////////////////////////////////////////////////////////

//BEGIN MetadataViewPrivate

//-----------------------------------------------------------------------------
class MetadataViewPrivate
{
public:
  void addItem(int id, QString const& keyText);
  void setItemValue(int id, QString const& valueText);
  void clearItemValue(int id);

  void removeItem(int id);

  QSet<int> itemIds() const { return this->keyLabels.keys().toSet(); }
  void updateLabelColors();

  QWidget* contentWidget;

protected:
  static void setKeyLabelColor(QLabel* label);

  static QString const valueTextTemplate;

  QHash<int, QLabel*> keyLabels;
  QHash<int, qtSqueezedLabel*> valueLabels;
};

//-----------------------------------------------------------------------------
void MetadataViewPrivate::addItem(int id, QString const& keyText)
{
  if (auto* const l = this->keyLabels.value(id, nullptr))
  {
    l->setText(keyText);
  }
  else
  {
    auto* const q = this->contentWidget;

    auto* const keyLabel = new QLabel{keyText, q};
    auto* const valueLabel = new qtSqueezedLabel{q};

    this->setKeyLabelColor(keyLabel);
    valueLabel->setTextMargins(1.0, 0.0);
    valueLabel->setElideMode(qtSqueezedLabel::ElideFade);

    auto* const layout = qobject_cast<QVBoxLayout*>(q->layout());
    if (layout->count())
    {
      delete layout->takeAt(layout->count() - 1);
    }
    layout->addWidget(keyLabel);
    layout->addWidget(valueLabel);
    layout->addStretch(1);

    keyLabel->show();
    valueLabel->show();

    q->resize(q->sizeHint());

    this->keyLabels.insert(id, keyLabel);
    this->valueLabels.insert(id, valueLabel);
    this->clearItemValue(id);
  }
}

//-----------------------------------------------------------------------------
void MetadataViewPrivate::setItemValue(int id, QString const& valueText)
{
  if (auto* const l = this->valueLabels.value(id, nullptr))
  {
    if (valueText.isEmpty())
    {
      l->setText("(empty)");
      l->setToolTip({});
      l->setEnabled(false);
    }
    else
    {
      l->setText(valueText.trimmed(), qtSqueezedLabel::SetToolTip);
      l->setEnabled(true);
    }
  }
}

//-----------------------------------------------------------------------------
void MetadataViewPrivate::clearItemValue(int id)
{
  if (auto* const l = this->valueLabels.value(id, nullptr))
  {
    l->setText("(not available)");
    l->setToolTip({});
    l->setEnabled(false);
  }
}

//-----------------------------------------------------------------------------
void MetadataViewPrivate::removeItem(int id)
{
  delete this->keyLabels.take(id);
  delete this->valueLabels.take(id);
}

//-----------------------------------------------------------------------------
void MetadataViewPrivate::setKeyLabelColor(QLabel* label)
{
  auto const& p = label->palette();
  auto const& bg = p.color(QPalette::Window);
  auto const& fg = p.color(QPalette::WindowText);
  auto const& c = qtColorUtil::blend(bg, fg, 0.6);
  label->setStyleSheet(QString{"color: %1;"}.arg(c.name()));
}

//-----------------------------------------------------------------------------
void MetadataViewPrivate::updateLabelColors()
{
  for (auto* const l : this->keyLabels)
  {
    this->setKeyLabelColor(l);
  }
}

//END MetadataViewPrivate

///////////////////////////////////////////////////////////////////////////////

//BEGIN MetadataView

//-----------------------------------------------------------------------------
MetadataView::MetadataView(QWidget* parent)
  : QScrollArea{parent}, d_ptr{new MetadataViewPrivate}
{
  QTE_D();

  d->contentWidget = new QWidget{this};
  d->contentWidget->setLayout(new QVBoxLayout);
  d->contentWidget->installEventFilter(this);

  this->setWidget(d->contentWidget);
}

//-----------------------------------------------------------------------------
MetadataView::~MetadataView()
{
}

//-----------------------------------------------------------------------------
bool MetadataView::eventFilter(QObject* sender, QEvent* e)
{
  QTE_D();

  if (sender == d->contentWidget && e && e->type() == QEvent::Resize)
  {
    this->setMinimumWidth(
      d->contentWidget->minimumSizeHint().width() +
      this->verticalScrollBar()->width());
  }

  return QScrollArea::eventFilter(sender, e);
}

//-----------------------------------------------------------------------------
void MetadataView::changeEvent(QEvent* e)
{
  if (e && e->type() == QEvent::PaletteChange)
  {
    QTE_D();
    d->updateLabelColors();
  }

  QScrollArea::changeEvent(e);
}

//-----------------------------------------------------------------------------
void MetadataView::updateMetadata(
  std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t> mdMap)
{
  QTE_D();

  QSet<kv::vital_metadata_tag> mdKeys;
  auto traits = kv::metadata_traits{};

  // Collect all keys present in the metadata map
  for (auto const& mdi : *mdMap)
  {
    for (auto const& mdp : mdi.second | kvr::valid)
    {
      for (auto const& mde : *mdp)
      {
        mdKeys.insert(mde.first);
      }
    }
  }

  // Update UI fields
  using md_tag_type_t = std::underlying_type<kv::vital_metadata_tag>::type;
  constexpr auto lastMetadataTag =
    static_cast<md_tag_type_t>(kv::VITAL_META_LAST_TAG);
  for (auto const k : qtIndexRange(lastMetadataTag))
  {
    auto const tag = static_cast<kv::vital_metadata_tag>(k);
    if (mdKeys.contains(tag))
    {
      d->clearItemValue(k);
      d->addItem(k, qtString(traits.tag_to_name(tag)));
    }
  }

  // Remove unused fields
  for (auto k : d->itemIds())
  {
    auto const tag = static_cast<kv::vital_metadata_tag>(k);
    if (!mdKeys.contains(tag))
    {
      d->removeItem(k);
    }
  }
}

//END MetadataView
