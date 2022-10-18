# Copyright 2022 The MediaPipe Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""MediaPipe Model Maker Python Public API For Image Classifier."""

from mediapipe.model_maker.python.vision.image_classifier import dataset
from mediapipe.model_maker.python.vision.image_classifier import hyperparameters
from mediapipe.model_maker.python.vision.image_classifier import image_classifier
from mediapipe.model_maker.python.vision.image_classifier import model_spec

ImageClassifier = image_classifier.ImageClassifier
HParams = hyperparameters.HParams
Dataset = dataset.Dataset
ModelSpec = model_spec.ModelSpec
SupportedModels = model_spec.SupportedModels
