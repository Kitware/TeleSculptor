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

#ifndef MAPTK_METADATAVIEW_H_
#define MAPTK_METADATAVIEW_H_

#include <vital/types/metadata_map.h>

#include <qtGlobal.h>

#include <QScrollArea>

class MetadataViewPrivate;

class MetadataView : public QScrollArea
{
  Q_OBJECT

public:
  explicit MetadataView(QWidget* parent = 0);
  ~MetadataView() override;

  bool eventFilter(QObject* sender, QEvent* e) override;

public slots:
  void updateMetadata(
    std::shared_ptr<kwiver::vital::metadata_map::map_metadata_t>);
  void updateMetadata(kwiver::vital::metadata_vector const&);

protected:
  void changeEvent(QEvent* e) override;

private:
  QTE_DECLARE_PRIVATE_RPTR(MetadataView)
  QTE_DECLARE_PRIVATE(MetadataView)

  QTE_DISABLE_COPY(MetadataView)
};

#endif
