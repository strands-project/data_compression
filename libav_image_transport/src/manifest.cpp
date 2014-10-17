/*
 *    manifest.cpp
 *
 *    Copyright 2013 Dominique Hunziker
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 *     \author/s: Dominique Hunziker
 */

#include <pluginlib/class_list_macros.h>

#include "libav_image_transport/libav_image_publisher.hpp"
#include "libav_image_transport/libav_image_subscriber.hpp"

PLUGINLIB_EXPORT_CLASS(libav_image_transport::LibAVImagePublisher, image_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS(libav_image_transport::LibAVImageSubscriber, image_transport::SubscriberPlugin)
