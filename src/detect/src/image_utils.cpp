// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "detect/image_utils.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "dnn/hb_sys.h"

static int32_t BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  if (height % 2 || width % 2) {
    std::cerr << "input img height and width must aligned by 2!";
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  if (yuv_mat.data == nullptr) {
    std::cerr << "yuv_mat.data is null pointer" << std::endl;
    return -1;
  }

  auto *yuv = yuv_mat.ptr<uint8_t>();
  if (yuv == nullptr) {
    std::cerr << "yuv is null pointer" << std::endl;
    return -1;
  }
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto *ynv12 = img_nv12.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  // copy y data
  int32_t y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  // copy uv data
  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }
  return 0;
}

std::shared_ptr<NV12PyramidInput> ImageUtils::GetNV12Pyramid(
    const cv::Mat &bgr_mat,
    int scaled_img_height,
    int scaled_img_width,
    float &scale,
    int &pad_left,
    int &pad_top) {
  if (bgr_mat.empty() || scaled_img_height <= 0 || scaled_img_width <= 0) {
    std::cerr << "invalid image or model input size" << std::endl;
    return nullptr;
  }

  if (scaled_img_height % 2 || scaled_img_width % 2) {
    std::cerr << "model input height and width must be aligned by 2!";
    return nullptr;
  }

  // Keep the source aspect ratio. Aurora RGB is 640x400, so stretching it
  // directly to the fixed model input would deform people and invalidate the
  // output coordinates.
  const float scale_w =
      static_cast<float>(scaled_img_width) / static_cast<float>(bgr_mat.cols);
  const float scale_h =
      static_cast<float>(scaled_img_height) / static_cast<float>(bgr_mat.rows);
  scale = std::min(scale_w, scale_h);

  const int resized_width =
      std::max(1, static_cast<int>(std::round(bgr_mat.cols * scale)));
  const int resized_height =
      std::max(1, static_cast<int>(std::round(bgr_mat.rows * scale)));

  pad_left = (scaled_img_width - resized_width) / 2;
  pad_top = (scaled_img_height - resized_height) / 2;

  cv::Mat resized;
  cv::resize(
      bgr_mat,
      resized,
      cv::Size(resized_width, resized_height),
      0,
      0,
      cv::INTER_LINEAR);

  // 114 is the conventional YOLO letterbox fill value.
  cv::Mat letterboxed(
      scaled_img_height,
      scaled_img_width,
      bgr_mat.type(),
      cv::Scalar(114, 114, 114));
  resized.copyTo(
      letterboxed(cv::Rect(pad_left, pad_top, resized_width, resized_height)));

  cv::Mat nv12_mat;
  auto ret = BGRToNv12(letterboxed, nv12_mat);
  if (ret) {
    std::cout << "get nv12 image failed " << std::endl;
    return nullptr;
  }

  auto *y = new hbSysMem;
  auto *uv = new hbSysMem;

  auto w_stride = ALIGN_16(scaled_img_width);
  hbSysAllocCachedMem(y, scaled_img_height * w_stride);
  hbSysAllocCachedMem(uv, scaled_img_height / 2 * w_stride);

  uint8_t *data = nv12_mat.data;
  auto *hb_y_addr = reinterpret_cast<uint8_t *>(y->virAddr);
  auto *hb_uv_addr = reinterpret_cast<uint8_t *>(uv->virAddr);

  // padding y
  for (int h = 0; h < scaled_img_height; ++h) {
    auto *raw = hb_y_addr + h * w_stride;
    for (int w = 0; w < scaled_img_width; ++w) {
      *raw++ = *data++;
    }
  }

  // padding uv
  auto uv_data = nv12_mat.data + scaled_img_height * scaled_img_width;
  for (int32_t h = 0; h < scaled_img_height / 2; ++h) {
    auto *raw = hb_uv_addr + h * w_stride;
    for (int32_t w = 0; w < scaled_img_width; ++w) {
      *raw++ = *uv_data++;
    }
  }

  hbSysFlushMem(y, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFlushMem(uv, HB_SYS_MEM_CACHE_CLEAN);
  auto pyramid = new NV12PyramidInput;
  pyramid->width = scaled_img_width;
  pyramid->height = scaled_img_height;
  pyramid->y_vir_addr = y->virAddr;
  pyramid->y_phy_addr = y->phyAddr;
  pyramid->y_stride = w_stride;
  pyramid->uv_vir_addr = uv->virAddr;
  pyramid->uv_phy_addr = uv->phyAddr;
  pyramid->uv_stride = w_stride;
  return std::shared_ptr<NV12PyramidInput>(pyramid,
                                           [y, uv](NV12PyramidInput *pyramid) {
                                             // Release memory after deletion
                                             hbSysFreeMem(y);
                                             hbSysFreeMem(uv);
                                             delete y;
                                             delete uv;
                                             delete pyramid;
                                           });
}
