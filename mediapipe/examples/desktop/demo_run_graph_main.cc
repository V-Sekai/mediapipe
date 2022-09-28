// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.
#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/classification.pb.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/osc/OscOutboundPacketStream.h"
#include "mediapipe/osc/ip/UdpSocket.h"
#include "mediapipe/Osc/OscSender.h"
#include "mediapipe/util/resource_util.h"

#define ADDRESS "127.0.0.1"
#define PORT 39539
#define OUTPUT_BUFFER_SIZE 1024

constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "output_video";
constexpr char kWindowName[] = "MediaPipe";
constexpr char kLandmarksStream[] = "pose_world_landmarks";
constexpr char kLeftHandLandmarksStream[] = "left_hand_landmarks";
constexpr char kRightHandLandmarksStream[] = "right_hand_landmarks";
constexpr char kFaceLandmarksStream[] = "face_landmarks_with_iris";

enum class MEDIAPIPE_POSE {
    MEDIAPIPE_NOSE = 0,
    MEDIAPIPE_LEFT_EYE_INNER = 1,
    MEDIAPIPE_LEFT_EYE = 2,
    MEDIAPIPE_LEFT_EYE_OUTER = 3,
    MEDIAPIPE_RIGHT_EYE_INNER = 4,
    MEDIAPIPE_RIGHT_EYE = 5,
    MEDIAPIPE_RIGHT_EYE_OUTER = 6,
    MEDIAPIPE_LEFT_EAR = 7,
    MEDIAPIPE_RIGHT_EAR = 8,
    MEDIAPIPE_MOUTH_LEFT = 9,
    MEDIAPIPE_MOUTH_RIGHT = 10,
    MEDIAPIPE_LEFT_SHOULDER = 11,
    MEDIAPIPE_RIGHT_SHOULDER = 12,
    MEDIAPIPE_LEFT_ELBOW = 13,
    MEDIAPIPE_RIGHT_ELBOW = 14,
    MEDIAPIPE_LEFT_WRIST = 15,
    MEDIAPIPE_RIGHT_WRIST = 16,
    MEDIAPIPE_LEFT_PINKY = 17,
    MEDIAPIPE_RIGHT_PINKY = 18,
    MEDIAPIPE_LEFT_INDEX = 19,
    MEDIAPIPE_RIGHT_INDEX = 20,
    MEDIAPIPE_LEFT_THUMB = 21,
    MEDIAPIPE_RIGHT_THUMB = 22,
    MEDIAPIPE_LEFT_HIP = 23,
    MEDIAPIPE_RIGHT_HIP = 24,
    MEDIAPIPE_LEFT_KNEE = 25,
    MEDIAPIPE_RIGHT_KNEE = 26,
    MEDIAPIPE_LEFT_ANKLE = 27,
    MEDIAPIPE_RIGHT_ANKLE = 28,
    MEDIAPIPE_LEFT_HEEL = 29,
    MEDIAPIPE_RIGHT_HEEL = 30,
    MEDIAPIPE_LEFT_FOOT_INDEX = 31,
    MEDIAPIPE_RIGHT_FOOT_INDEX = 32,
};

ABSL_FLAG(std::string, calculator_graph_config_file, "",
          "Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(std::string, input_video_path, "",
          "Full path of video to load. "
          "If not provided, attempt to use a webcam.");
ABSL_FLAG(std::string, output_video_path, "",
          "Full path of where to save result (.mp4 only). "
          "If not provided, show result in a window.");

absl::Status RunMPPGraph() {
  UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
  char buffer[OUTPUT_BUFFER_SIZE];
  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
      absl::GetFlag(FLAGS_calculator_graph_config_file),
      &calculator_graph_config_contents));
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  LOG(INFO) << "Initialize the camera or load the video.";
  cv::VideoCapture capture;
  const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
  if (load_video) {
    capture.open(absl::GetFlag(FLAGS_input_video_path));
  } else {
    capture.open(0);
  }
  RET_CHECK(capture.isOpened());

  cv::VideoWriter writer;
  const bool save_video = !absl::GetFlag(FLAGS_output_video_path).empty();
  if (!save_video) {
    cv::namedWindow(kWindowName, 1);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capture.set(cv::CAP_PROP_FPS, 60);
  }

  LOG(INFO) << "Start running the calculator graph.";
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                   graph.AddOutputStreamPoller(kOutputStream));

  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_landmarks,
                   graph.AddOutputStreamPoller(kLandmarksStream));

  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller left_poller_landmarks,
                   graph.AddOutputStreamPoller(kLeftHandLandmarksStream));

  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller right_poller_landmarks,
                   graph.AddOutputStreamPoller(kRightHandLandmarksStream));

  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller face_poller_landmarks,
                   graph.AddOutputStreamPoller(kFaceLandmarksStream));

  MP_RETURN_IF_ERROR(graph.StartRun({}));

  enum {
    LOADED_NOT_LOADED = 0,
    LOADED_LOADED = 1,
  };
  enum {
    CALIBRATION_STATE_UNCALIBRATED = 0,
    CALIBRATION_STATE_WAITING_FOR_CALIBRATION = 1,
    CALIBRATION_STATE_CALIBRATING = 2,
    CALIBRATION_STATE_CALIBRATED = 3,
  };
  enum {
    CALIBRATION_MODE_NORMAL = 0,
    CALIBRATION_MODE_MR_NORMAL = 1,
    CALIBRATION_MODE_MR_FLOOR_FIX = 2,
  };
  int32_t loaded = LOADED_LOADED;
  int32_t calibration_state = CALIBRATION_STATE_CALIBRATED;
  int32_t calibration_mode = CALIBRATION_MODE_NORMAL;


  LOG(INFO) << "Start grabbing and processing frames.";
  bool grab_frames = true;
  while (grab_frames) {
    // Capture opencv camera or video frame.
    cv::Mat camera_frame_raw;
    capture >> camera_frame_raw;
    if (camera_frame_raw.empty()) {
      if (!load_video) {
        LOG(INFO) << "Ignore empty frames from camera.";
        continue;
      }
      LOG(INFO) << "Empty frame, end of video reached.";
      break;
    }
    cv::Mat camera_frame;
    cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGB);
    if (!load_video) {
      cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
    }

    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
        mediapipe::ImageFrame::kDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);

    // Send image packet into the graph.
    size_t frame_timestamp_us =
        (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
    MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
        kInputStream, mediapipe::Adopt(input_frame.release())
                          .At(mediapipe::Timestamp(frame_timestamp_us))));

    // Get the graph result packet and stop if it fails.
    mediapipe::Packet packet;
    if (!poller.Next(&packet))
      break;

    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
      p << osc::BeginBundleImmediate << osc::BeginMessage( "/VMC/Ext/OK" ) << loaded \
      << calibration_state << calibration_mode << osc::EndMessage << osc::EndBundle;
    transmitSocket.Send( p.Data(), p.Size() );
    
    mediapipe::Packet landmark_packet;

    if (poller_landmarks.QueueSize() &&
        poller_landmarks.Next(&landmark_packet)) {
      auto &landmarkList =
          landmark_packet.Get<::mediapipe::LandmarkList>();

      for (int j = 0; j < landmarkList.landmark_size(); j++) {
        auto &landmark = landmarkList.landmark(j);
          osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
          p << osc::BeginBundleImmediate << osc::BeginMessage( "/VMC/Ext/Bone/Pos" ) 
            << (std::string("pose_") + std::to_string(j)).c_str() << 
              (float)landmark.x() << (float)landmark.y() << (float)landmark.z() << 
              (float) 0.0 << (float)0.0  << (float)0.0  << (float)1.0 << 
              (float) 1.0 << (float)1.0  << (float)1.0  << 
              (float) 1.0 << (float)1.0  << (float)1.0  << 
              osc::EndMessage
        << osc::EndBundle;
        transmitSocket.Send( p.Data(), p.Size() );
      }
    }

    mediapipe::Packet face_landmark_packet;

    if (face_poller_landmarks.QueueSize() &&
        face_poller_landmarks.Next(&face_landmark_packet)) {
      auto &landmarkList =
          face_landmark_packet.Get<::mediapipe::NormalizedLandmarkList>();

      for (int j = 0; j < landmarkList.landmark_size(); j++) {
        auto &landmark = landmarkList.landmark(j);
          osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
          p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/VMC/Ext/Bone/Pos" ) 
            << (std::string("face_") + std::to_string(j)).c_str() << 
              (float)landmark.x() << (float)landmark.y() << (float)landmark.z() << 
              (float) 0.0 << (float)0.0  << (float)0.0  << (float)1.0 << 
              (float) 1.0 << (float)1.0  << (float)1.0  << 
              (float) 1.0 << (float)1.0  << (float)1.0  << 
              osc::EndMessage
        << osc::EndBundle;
        transmitSocket.Send( p.Data(), p.Size() );
      }
    }

    mediapipe::Packet left_landmark_packet;

    if (left_poller_landmarks.QueueSize() &&
        left_poller_landmarks.Next(&left_landmark_packet)) {
      auto &landmarkList =
          left_landmark_packet.Get<::mediapipe::NormalizedLandmarkList>();

      for (int j = 0; j < landmarkList.landmark_size(); j++) {
        auto &landmark = landmarkList.landmark(j);
        osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
        p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/VMC/Ext/Bone/Pos" ) << (std::string("left_hand_") + std::to_string(j)).c_str() << \
              (float)landmark.x() << (float)landmark.y() << (float)landmark.z() << \
              (float) 0.0 << (float)0.0  << (float)0.0  << (float)1.0 << \
              (float) 1.0 << (float)1.0  << (float)1.0  << \
              (float) 1.0 << (float)1.0  << (float)1.0  << \
              osc::EndMessage
        << osc::EndBundle;
        transmitSocket.Send( p.Data(), p.Size() );
      }
    }
    mediapipe::Packet right_landmark_packet;
    if (right_poller_landmarks.QueueSize() &&
        right_poller_landmarks.Next(&right_landmark_packet)) {
      auto &landmarkList =
          right_landmark_packet.Get<::mediapipe::NormalizedLandmarkList>();
      for (int j = 0; j < landmarkList.landmark_size(); j++) {
        auto &landmark = landmarkList.landmark(j);
        osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
        p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/VMC/Ext/Bone/Pos" ) 
            << (std::string("right_hand_") + std::to_string(j)).c_str() << 
              (float)landmark.x() << (float)landmark.y() << (float)landmark.z() << 
              (float) 0.0 << (float)0.0  << (float)0.0  << (float)1.0 << 
              (float) 1.0 << (float)1.0  << (float)1.0  << 
              (float) 1.0 << (float)1.0  << (float)1.0  << 
              osc::EndMessage
        << osc::EndBundle;
        transmitSocket.Send( p.Data(), p.Size() );
      }
    }

    auto &output_frame = packet.Get<mediapipe::ImageFrame>();
    // Convert back to opencv for display or saving.
    cv::Mat output_frame_mat = mediapipe::formats::MatView(&output_frame);
    cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
    if (save_video) {
      if (!writer.isOpened()) {
        LOG(INFO) << "Prepare video writer.";
        writer.open(absl::GetFlag(FLAGS_output_video_path),
                    mediapipe::fourcc('a', 'v', 'c', '1'), // .mp4
                    capture.get(cv::CAP_PROP_FPS), output_frame_mat.size());
        RET_CHECK(writer.isOpened());
      }
      writer.write(output_frame_mat);
    } else {
      cv::imshow(kWindowName, output_frame_mat);
      // Press space key to exit.
      const int pressed_key = cv::waitKey(5);
      if (pressed_key >= 0 && pressed_key != 255)
        grab_frames = false;
    }
  }

  LOG(INFO) << "Shutting down.";
  if (writer.isOpened())
    writer.release();
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  return graph.WaitUntilDone();
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  absl::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}
